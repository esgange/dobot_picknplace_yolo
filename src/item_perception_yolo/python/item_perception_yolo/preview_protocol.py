"""Bounded JSON/byte transport; deliberately no native inference imports."""

import json
import struct

MAX_HEADER = 4 * 1024 * 1024
MAX_IMAGE_BYTES = 4096 * 4096 * 6


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        block = stream.read(size - len(data))
        if not block:
            raise EOFError("Inference worker pipe closed")
        data.extend(block)
    return bytes(data)


def send_packet(stream, header, data=b""):
    if len(data) > MAX_IMAGE_BYTES:
        raise ValueError("Preview frame exceeds bounded transport size")
    encoded = json.dumps({**header, "byte_count": len(data)}, allow_nan=False).encode("utf-8")
    if len(encoded) > MAX_HEADER:
        raise ValueError("Preview header exceeds bounded transport size")
    stream.write(struct.pack("!I", len(encoded)) + encoded + data)
    stream.flush()


def receive_packet(stream):
    size = struct.unpack("!I", read_exact(stream, 4))[0]
    if not 0 < size <= MAX_HEADER:
        raise ValueError("Malformed preview header size")
    header = json.loads(read_exact(stream, size))
    if type(header) is not dict:
        raise ValueError("Malformed preview header")
    size = header.pop("byte_count", None)
    if type(size) is not int or not 0 <= size <= MAX_IMAGE_BYTES:
        raise ValueError("Malformed preview payload size")
    return header, read_exact(stream, size)
