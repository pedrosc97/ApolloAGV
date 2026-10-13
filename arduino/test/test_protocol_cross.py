"""Cross-language check: the generated C++ header and Python module agree on the wire.

Compiles a small host program against arduino/ring_stepper/ring_protocol.h with the
desktop g++ (no Arduino needed), then
  1. compares the frames C++ encodes with the frames Python encodes, byte for byte;
  2. feeds Python-encoded frames, with noise, through the C++ parser.
Skipped if g++ is not installed.  Run:  python3 -m pytest arduino/test
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "colcon" / "ros2_ws" / "src" / "apollo_ring"))
from apollo_ring import ring_protocol as rp  # noqa: E402

CPP = r"""
#include <cstdio>
#include "ring_protocol.h"
using namespace ring;

static void hex(const uint8_t* b, uint8_t n) { for (uint8_t i = 0; i < n; i++) printf("%02x", b[i]); printf("\n"); }

int main() {
  uint8_t out[MAX_FRAME];
  CmdVelMsg v{1.5f};                      hex(out, encode(v, out));
  hex(out, encode(CMD_HOME, nullptr, 0, out));
  CmdEnableMsg e{1};                      hex(out, encode(e, out));
  StatusMsg s{0.25f, -0.5f, READY, F_ENABLED | F_HOME_SW, 0.01f};
  hex(out, encode(s, out));
  // Parse whatever arrives on stdin and print one line per good frame.
  Parser p;
  int c;
  while ((c = getchar()) != EOF) {
    if (!p.feed((uint8_t)c)) continue;
    printf("frame %02x %u", p.type, p.len);
    StatusMsg st;
    if (p.type == STATUS && unpack(p.payload, p.len, st)) printf(" state=%u flags=%u", st.state, st.flags);
    printf("\n");
  }
  return 0;
}
"""


@pytest.fixture(scope="module")
def host_bin(tmp_path_factory):
    if not shutil.which("g++"):
        pytest.skip("g++ not installed")
    d = tmp_path_factory.mktemp("cross")
    (d / "main.cpp").write_text(CPP)
    exe = d / "cross"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Werror", "-I", str(ROOT / "arduino" / "ring_stepper"),
                    str(d / "main.cpp"), "-o", str(exe)], check=True)
    return exe


def run(exe, stdin=b""):
    return subprocess.run([str(exe)], input=stdin, capture_output=True, check=True).stdout.decode().split("\n")


def test_cpp_encodes_like_python(host_bin):
    lines = run(host_bin)
    expected = [
        rp.CmdVel(1.5).encode(),
        rp.CmdHome().encode(),
        rp.CmdEnable(1).encode(),
        rp.Status(0.25, -0.5, rp.State.READY, rp.Flags.ENABLED | rp.Flags.HOME_SW, 0.01).encode(),
    ]
    assert lines[:4] == [f.hex() for f in expected]


def test_cpp_parses_python_frames(host_bin):
    st = rp.Status(0.0, 0.0, rp.State.HOMING, rp.Flags.WATCHDOG, 0.0).encode()
    bad = bytearray(rp.CmdVel(3.0).encode()); bad[-1] ^= 1
    stream = b"\xaa\xaa\x55" + bytes(bad) + rp.CmdVel(3.0).encode() + b"\x00" + st
    frames = [l for l in run(host_bin, stream) if l.startswith("frame")]
    assert frames == ["frame 01 4", "frame 81 14 state=1 flags=2"]
