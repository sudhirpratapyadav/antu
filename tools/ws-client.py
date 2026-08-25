#!/usr/bin/env python3
"""
A minimal WebSocket client for the antu bridge. Standard library only.

Enough of RFC 6455 to talk to the bridge: the handshake, masked text frames out,
unmasked frames in, and ping/pong. Exists so the bridge can be exercised from a
laptop without installing anything on either end.

    ./tools/ws-client.py <host> watch /odom /base/status
    ./tools/ws-client.py <host> topics
    ./tools/ws-client.py <host> drive 0.0 0.35 3      # linearX rad/s seconds
"""
import base64, json, os, socket, struct, sys, time

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class Ws:
    def __init__(self, host, port=8080, path="/ws"):
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n").encode())

        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise IOError("server closed during handshake")
            buf += chunk
        head, _, rest = buf.partition(b"\r\n\r\n")
        if b"101" not in head.split(b"\r\n")[0]:
            raise IOError("handshake refused: " + head.decode(errors="replace")[:200])
        self.buf = rest

    def send(self, obj):
        payload = json.dumps(obj).encode()
        header = bytearray([0x81])                  # FIN + text
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n <= 0xFFFF:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        header += mask
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise IOError("connection closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        """Next text message as a dict, or None when the peer closes."""
        while True:
            b0, b1 = self._read(2)
            opcode = b0 & 0x0F
            length = b1 & 0x7F
            if length == 126:
                length = struct.unpack(">H", self._read(2))[0]
            elif length == 127:
                length = struct.unpack(">Q", self._read(8))[0]
            if b1 & 0x80:                            # server should never mask
                self._read(4)
            payload = self._read(length)
            if opcode == 0x8:
                return None
            if opcode == 0x9:                        # ping -> pong
                continue
            if opcode in (0x1, 0x0):
                return json.loads(payload.decode())

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    cmd = sys.argv[2] if len(sys.argv) > 2 else "topics"
    ws = Ws(host)
    try:
        first = ws.recv()                            # catalogue arrives unprompted

        if cmd == "topics":
            for t in first["payload"]["topics"]:
                print("%-16s %-12s n=%-8s writable=%s"
                      % (t["name"], t["type"], t["published"], t["writable"]))

        elif cmd == "watch":
            topics = sys.argv[3:] or ["/odom"]
            ws.send({"type": "subscribe",
                     "payload": {"topics": topics, "maxHz": 5}})
            end = time.time() + 6
            while time.time() < end:
                msg = ws.recv()
                if msg is None:
                    break
                if msg["type"] == "snapshot":
                    for m in msg["payload"]["topics"]:
                        print("snapshot %-14s %s" % (m["topic"], json.dumps(m["value"])[:110]))
                elif msg["type"] == "msg":
                    m = msg["payload"]
                    print("msg      %-14s seq=%-5s %s"
                          % (m["topic"], m["seq"], json.dumps(m["value"])[:110]))
                else:
                    print(msg)

        elif cmd == "drive":
            linear = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
            angular = float(sys.argv[4]) if len(sys.argv) > 4 else 0.3
            secs = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
            ws.send({"type": "subscribe", "payload": {"topics": ["/odom"], "maxHz": 5}})
            end = time.time() + secs
            while time.time() < end:
                ws.send({"type": "publish", "payload": {
                    "topic": "/cmd_vel",
                    "value": {"linearX": linear, "angular": angular}}})
                time.sleep(0.1)
                ws.sock.settimeout(0.2)
                try:
                    msg = ws.recv()
                    if msg and msg["type"] == "msg":
                        v = msg["payload"]["value"]
                        print("theta=%7.3f  vel=%s" % (v["pose"]["theta"], v["velocity"]))
                except (socket.timeout, OSError):
                    pass
            ws.send({"type": "publish", "payload": {
                "topic": "/cmd_vel", "value": {"linearX": 0, "angular": 0}}})
            print("stopped")
    finally:
        ws.close()


if __name__ == "__main__":
    main()
