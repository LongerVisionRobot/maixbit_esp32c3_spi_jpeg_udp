import socket, struct

PORT = 5006
HDR_LEN = 10
FLAG_START = 1
FLAG_END = 2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
print("[pc] listening", PORT)

cur = {}  # frame_id -> bytearray

while True:
    data, _ = sock.recvfrom(4096)
    if len(data) < HDR_LEN:
        continue

    frame_id, chunk_id, flags, rsv, payload_len = struct.unpack_from(
        "<I H B B H", data, 0
    )
    payload = data[HDR_LEN : HDR_LEN + payload_len]

    if flags & FLAG_START:
        cur[frame_id] = bytearray()

    if frame_id not in cur:
        # haven't seen START; drop
        continue

    cur[frame_id].extend(payload)

    if flags & FLAG_END:
        jpg = cur.pop(frame_id)
        fn = f"latest.jpg"
        with open(fn, "wb") as f:
            f.write(jpg)
        print(f"[pc] wrote {fn} frame_id={frame_id} bytes={len(jpg)}")
