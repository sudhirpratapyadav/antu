package com.antu.ops;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Base64;
import java.util.Map;

/**
 * One WebSocket connection: the RFC 6455 handshake and frame codec.
 *
 * <p>Written out rather than pulled in, for the same reason as everything else
 * here — the ops module has no dependencies, and the subset a telemetry bridge
 * needs is small: text frames, close, and ping/pong. Binary and extensions are
 * not implemented, and continuation frames are reassembled but capped.
 *
 * <p>Chosen over server-sent events because control has to flow both ways. SSE
 * would be half the code and cover telemetry, but then commands need a second
 * channel over plain HTTP, and the two get out of step about whether a client is
 * still there.
 *
 * <p>Sending is synchronised, so any thread may write; the bus publishes from the
 * graph's tick thread while the reader sits in a blocking read.
 */
public final class WebSocket {

    /** The fixed GUID RFC 6455 defines for the accept-key hash. */
    private static final String GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

    private static final int OP_CONTINUATION = 0x0;
    private static final int OP_TEXT = 0x1;
    private static final int OP_BINARY = 0x2;
    private static final int OP_CLOSE = 0x8;
    private static final int OP_PING = 0x9;
    private static final int OP_PONG = 0xA;

    /**
     * Largest message accepted from a client.
     *
     * <p>Commands are tiny. A frame claiming more than this is either a bug or an
     * attempt to make the phone allocate a gigabyte, and the length field is 64
     * bits wide, so it must be checked before allocating anything.
     */
    private static final int MAX_MESSAGE_BYTES = 256 * 1024;

    private final Socket socket;
    private final InputStream in;
    private final OutputStream out;
    private final Object sendLock = new Object();
    private volatile boolean open = true;

    private WebSocket(Socket socket) throws IOException {
        this.socket = socket;
        this.in = socket.getInputStream();
        this.out = socket.getOutputStream();
    }

    /** True when the request is a WebSocket upgrade. */
    public static boolean isUpgrade(Map<String, String> headers) {
        String upgrade = headers.get("upgrade");
        return upgrade != null && upgrade.toLowerCase().contains("websocket");
    }

    /**
     * Completes the handshake and returns the live connection.
     *
     * @throws IOException if the client did not send a usable key
     */
    public static WebSocket accept(Socket socket, Map<String, String> headers) throws IOException {
        String key = headers.get("sec-websocket-key");
        if (key == null) {
            throw new IOException("missing Sec-WebSocket-Key");
        }
        String accept = Base64.getEncoder().encodeToString(sha1(key + GUID));
        String response = "HTTP/1.1 101 Switching Protocols\r\n"
                + "Upgrade: websocket\r\n"
                + "Connection: Upgrade\r\n"
                + "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
        OutputStream out = socket.getOutputStream();
        out.write(response.getBytes("UTF-8"));
        out.flush();
        // No read timeout once upgraded: a telemetry connection is legitimately
        // idle between commands, and treating that as a dead client would drop
        // every viewer that is only watching.
        socket.setSoTimeout(0);
        return new WebSocket(socket);
    }

    public boolean isOpen() {
        return open && !socket.isClosed();
    }

    /** Sends a text message. Safe from any thread. */
    public void send(String message) throws IOException {
        byte[] payload = message.getBytes("UTF-8");
        synchronized (sendLock) {
            writeFrame(OP_TEXT, payload);
        }
    }

    /**
     * Reads the next text message, blocking until one arrives.
     *
     * <p>Control frames are handled here and never returned: a ping is answered
     * with a pong, a close is echoed and ends the stream.
     *
     * @return the message, or null when the connection has closed
     */
    public String receive() throws IOException {
        java.io.ByteArrayOutputStream assembled = null;
        int assembledOpcode = -1;

        while (isOpen()) {
            Frame frame = readFrame();
            if (frame == null) {
                close();
                return null;
            }

            switch (frame.opcode) {
                case OP_CLOSE:
                    synchronized (sendLock) {
                        writeFrame(OP_CLOSE, new byte[0]);
                    }
                    close();
                    return null;

                case OP_PING:
                    synchronized (sendLock) {
                        writeFrame(OP_PONG, frame.payload);
                    }
                    continue;

                case OP_PONG:
                    continue;                        // unsolicited pongs are legal

                case OP_BINARY:
                    // Nothing here speaks binary; skip rather than misread it as text.
                    continue;

                case OP_TEXT:
                case OP_CONTINUATION:
                    if (frame.opcode == OP_TEXT && assembled == null) {
                        if (frame.fin) {
                            return new String(frame.payload, "UTF-8");
                        }
                        assembled = new java.io.ByteArrayOutputStream();
                        assembledOpcode = OP_TEXT;
                    }
                    if (assembled == null) {
                        continue;                    // continuation with nothing started
                    }
                    assembled.write(frame.payload);
                    if (assembled.size() > MAX_MESSAGE_BYTES) {
                        throw new IOException("message exceeds " + MAX_MESSAGE_BYTES + " bytes");
                    }
                    if (frame.fin) {
                        String text = new String(assembled.toByteArray(), "UTF-8");
                        assembled = null;
                        if (assembledOpcode == OP_TEXT) {
                            return text;
                        }
                    }
                    continue;

                default:
                    throw new IOException("unsupported opcode: " + frame.opcode);
            }
        }
        return null;
    }

    public void close() {
        open = false;
        try {
            socket.close();
        } catch (IOException e) {
            // Already gone.
        }
    }

    // ---------- framing ----------

    private static final class Frame {
        final boolean fin;
        final int opcode;
        final byte[] payload;

        Frame(boolean fin, int opcode, byte[] payload) {
            this.fin = fin;
            this.opcode = opcode;
            this.payload = payload;
        }
    }

    private Frame readFrame() throws IOException {
        int b0 = in.read();
        if (b0 < 0) {
            return null;
        }
        int b1 = in.read();
        if (b1 < 0) {
            return null;
        }

        boolean fin = (b0 & 0x80) != 0;
        int opcode = b0 & 0x0F;
        boolean masked = (b1 & 0x80) != 0;
        long length = b1 & 0x7F;

        if (length == 126) {
            length = ((long) readByte() << 8) | readByte();
        } else if (length == 127) {
            length = 0;
            for (int i = 0; i < 8; i++) {
                length = (length << 8) | readByte();
            }
        }
        // Checked before allocating: the field is 64 bits, and a client is free
        // to claim any of it.
        if (length < 0 || length > MAX_MESSAGE_BYTES) {
            throw new IOException("frame too large: " + length);
        }

        byte[] mask = null;
        if (masked) {
            mask = new byte[4];
            readFully(mask);
        } else {
            // RFC 6455: a client must mask. An unmasked frame is a broken or
            // hostile client, and unmasking it anyway hides the problem.
            throw new IOException("client frame was not masked");
        }

        byte[] payload = new byte[(int) length];
        readFully(payload);
        for (int i = 0; i < payload.length; i++) {
            payload[i] ^= mask[i & 3];
        }
        return new Frame(fin, opcode, payload);
    }

    /** Must hold {@link #sendLock}. Server frames are never masked. */
    private void writeFrame(int opcode, byte[] payload) throws IOException {
        java.io.ByteArrayOutputStream frame = new java.io.ByteArrayOutputStream(payload.length + 10);
        frame.write(0x80 | opcode);                  // FIN set: no fragmentation on send

        int length = payload.length;
        if (length < 126) {
            frame.write(length);
        } else if (length <= 0xFFFF) {
            frame.write(126);
            frame.write((length >> 8) & 0xFF);
            frame.write(length & 0xFF);
        } else {
            frame.write(127);
            for (int shift = 56; shift >= 0; shift -= 8) {
                frame.write((int) (((long) length >> shift) & 0xFF));
            }
        }
        frame.write(payload, 0, payload.length);

        // One write, so two threads cannot interleave halves of a frame even if
        // the lock were ever lost.
        out.write(frame.toByteArray());
        out.flush();
    }

    private int readByte() throws IOException {
        int b = in.read();
        if (b < 0) {
            throw new IOException("connection closed mid-frame");
        }
        return b;
    }

    private void readFully(byte[] dst) throws IOException {
        int off = 0;
        while (off < dst.length) {
            int n = in.read(dst, off, dst.length - off);
            if (n < 0) {
                throw new IOException("connection closed mid-frame");
            }
            off += n;
        }
    }

    private static byte[] sha1(String s) throws IOException {
        try {
            return MessageDigest.getInstance("SHA-1").digest(s.getBytes("UTF-8"));
        } catch (NoSuchAlgorithmException e) {
            throw new IOException("SHA-1 unavailable", e);
        }
    }
}
