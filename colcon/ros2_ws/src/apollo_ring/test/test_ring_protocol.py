"""Python side of the ring protocol: CRC, framing, round trips, parser resync."""
import struct

from apollo_ring import ring_protocol as rp


def test_crc8_maxim_check_value():
    assert rp.crc8(b"123456789") == 0xA1  # standard CRC-8/MAXIM check value


def test_cmd_vel_frame_bytes():
    frame = rp.CmdVel(1.5).encode()
    body = bytes([rp.CMD_VEL, 4]) + struct.pack("<f", 1.5)
    assert frame == b"\xaa\x55" + body + bytes([rp.crc8(body)])
    assert len(frame) == rp.CmdVel.LEN + 5


def test_status_round_trip():
    s = rp.Status(0.25, -0.5, rp.State.READY, rp.Flags.ENABLED | rp.Flags.HOME_SW, 0.01)
    (t, p), = rp.Parser().feed(s.encode())
    out = rp.decode(t, p)
    assert isinstance(out, rp.Status)
    assert out.state is rp.State.READY and out.flags == s.flags
    assert abs(out.angle - 0.25) < 1e-6 and abs(out.cross_error - 0.01) < 1e-6


def test_empty_message():
    assert rp.CmdHome().encode() == rp.encode(rp.CMD_HOME)
    assert rp.decode(rp.CMD_HOME, b"") == rp.CmdHome()


def test_parser_skips_noise_bad_crc_and_split_frames():
    good = rp.CmdEnable(1).encode()
    bad = bytearray(rp.CmdVel(2.0).encode())
    bad[-1] ^= 0xFF
    stream = b"\x00\xaa\x13" + bytes(bad) + good + good
    parser = rp.Parser()
    got = []
    for i in range(0, len(stream), 3):  # arrive in small chunks
        got += list(parser.messages(stream[i:i + 3]))
    assert got == [rp.CmdEnable(1), rp.CmdEnable(1)]
    assert parser.crc_errors == 1


def test_decode_rejects_wrong_length_and_unknown_type():
    assert rp.decode(rp.STATUS, b"\x00" * 3) is None
    assert rp.decode(0x7F, b"") is None
