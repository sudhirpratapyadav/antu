package com.antu.ops;

import com.antu.core.log.Log;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;

/**
 * A small HTTP server for the operations layer.
 *
 * <p>Plain {@code java.net}, no Android: the whole ops layer runs on a desktop
 * against a recording, which is the point of it being a pure module.
 *
 * <p>Sized for what it is — a handful of developers and a browser tab, not
 * public traffic. A small fixed pool rather than a thread per connection, so a
 * client that opens sockets and walks away cannot exhaust the phone.
 */
public final class HttpServer {

    /** Enough for a browser's parallel requests plus a couple of curl sessions. */
    private static final int WORKERS = 4;
    /** Drop a client that opens a socket and says nothing. */
    private static final int READ_TIMEOUT_MS = 15000;
    /** Refuse a request line longer than this rather than buffering it. */
    private static final int MAX_REQUEST_LINE = 8192;

    /** Handles one request. */
    public interface Handler {
        Response handle(Request request) throws Exception;
    }

    /** What a handler is given. */
    public static final class Request {
        public final String method;
        public final String path;
        public final Map<String, String> query;

        Request(String method, String path, Map<String, String> query) {
            this.method = method;
            this.path = path;
            this.query = query;
        }

        /** Query parameter, or {@code fallback} when absent or unparseable. */
        public double number(String key, double fallback) {
            String v = query.get(key);
            if (v == null) {
                return fallback;
            }
            try {
                return Double.parseDouble(v);
            } catch (NumberFormatException e) {
                return fallback;
            }
        }

        public String text(String key, String fallback) {
            String v = query.get(key);
            return v == null ? fallback : v;
        }

        public boolean flag(String key, boolean fallback) {
            String v = query.get(key);
            if (v == null) {
                return fallback;
            }
            return !("0".equals(v) || "false".equalsIgnoreCase(v));
        }
    }

    /** What a handler returns. */
    public static final class Response {
        final int status;
        final String contentType;
        final byte[] body;

        private Response(int status, String contentType, byte[] body) {
            this.status = status;
            this.contentType = contentType;
            this.body = body;
        }

        public static Response json(String body) {
            return new Response(200, "application/json", utf8(body));
        }

        public static Response text(String body) {
            return new Response(200, "text/plain", utf8(body));
        }

        public static Response html(String body) {
            return new Response(200, "text/html", utf8(body));
        }

        public static Response bytes(String contentType, byte[] body) {
            return new Response(200, contentType, body);
        }

        public static Response notFound(String message) {
            return new Response(404, "text/plain", utf8(message + "\n"));
        }

        public static Response error(int status, String message) {
            return new Response(status, "text/plain", utf8(message + "\n"));
        }

        private static byte[] utf8(String s) {
            try {
                return s.getBytes("UTF-8");
            } catch (java.io.UnsupportedEncodingException e) {
                throw new IllegalStateException("UTF-8 is always supported", e);
            }
        }
    }

    private static final String TAG = "http";

    private final int port;
    private final Map<String, Handler> routes = new LinkedHashMap<>();
    private Handler fallback;

    private ServerSocket server;
    private ExecutorService workers;
    private volatile boolean running;

    public HttpServer(int port) {
        this.port = port;
    }

    /** Registers a handler for an exact path. */
    public HttpServer route(String path, Handler handler) {
        routes.put(path, handler);
        return this;
    }

    /** Handles anything unmatched — static assets, usually. */
    public HttpServer fallback(Handler handler) {
        this.fallback = handler;
        return this;
    }

    public int port() {
        return port;
    }

    public boolean isRunning() {
        return running;
    }

    public void start() throws IOException {
        if (running) {
            return;
        }
        server = new ServerSocket(port);
        running = true;
        workers = Executors.newFixedThreadPool(WORKERS, new ThreadFactory() {
            private int n;

            @Override public Thread newThread(Runnable r) {
                Thread t = new Thread(r, "antu-http-" + (++n));
                t.setDaemon(true);
                return t;
            }
        });
        Thread acceptor = new Thread(this::accept, "antu-http-accept");
        acceptor.setDaemon(true);
        acceptor.start();
        Log.i(TAG, "listening on " + localAddress() + ":" + port);
    }

    public void stop() {
        running = false;
        try {
            if (server != null) {
                server.close();
            }
        } catch (IOException e) {
            // Shutting down; nothing to do.
        }
        if (workers != null) {
            workers.shutdownNow();
        }
    }

    private void accept() {
        while (running) {
            try {
                Socket socket = server.accept();
                workers.execute(() -> serve(socket));
            } catch (IOException e) {
                if (running) {
                    Log.w(TAG, "accept failed: " + e.getMessage());
                }
            } catch (java.util.concurrent.RejectedExecutionException e) {
                // Shutting down between accept and execute.
                return;
            }
        }
    }

    private void serve(Socket socket) {
        try {
            socket.setSoTimeout(READ_TIMEOUT_MS);
            InputStream in = socket.getInputStream();
            String line = readLine(in);
            if (line == null || line.isEmpty()) {
                return;
            }
            String[] parts = line.split(" ");
            if (parts.length < 2) {
                respond(socket, Response.error(400, "malformed request"));
                return;
            }
            // Headers are read and discarded: nothing here needs them, but they
            // must be consumed or the client sees a reset before its body lands.
            while (true) {
                String header = readLine(in);
                if (header == null || header.isEmpty()) {
                    break;
                }
            }

            Request request = parse(parts[0], parts[1]);
            Handler handler = routes.get(request.path);
            if (handler == null) {
                handler = fallback;
            }
            if (handler == null) {
                respond(socket, Response.notFound("no such endpoint: " + request.path));
                return;
            }
            respond(socket, handler.handle(request));
        } catch (Exception e) {
            try {
                respond(socket, Response.error(500, "handler failed: " + e));
            } catch (IOException ignored) {
                // Client is gone.
            }
            Log.w(TAG, "request failed: " + e);
        } finally {
            try {
                socket.close();
            } catch (IOException ignored) {
                // Already closed.
            }
        }
    }

    private static Request parse(String method, String target) {
        String path = target;
        Map<String, String> query = new HashMap<>();
        int q = target.indexOf('?');
        if (q >= 0) {
            path = target.substring(0, q);
            for (String pair : target.substring(q + 1).split("&")) {
                if (pair.isEmpty()) {
                    continue;
                }
                int eq = pair.indexOf('=');
                String key = eq < 0 ? pair : pair.substring(0, eq);
                String value = eq < 0 ? "" : pair.substring(eq + 1);
                query.put(decode(key), decode(value));
            }
        }
        return new Request(method, decode(path), query);
    }

    private static String decode(String s) {
        try {
            return URLDecoder.decode(s, "UTF-8");
        } catch (Exception e) {
            return s;
        }
    }

    private static String readLine(InputStream in) throws IOException {
        ByteArrayOutputStream buf = new ByteArrayOutputStream();
        int c;
        while ((c = in.read()) != -1) {
            if (c == '\n') {
                break;
            }
            if (c != '\r') {
                buf.write(c);
            }
            if (buf.size() > MAX_REQUEST_LINE) {
                throw new IOException("request line too long");
            }
        }
        if (c == -1 && buf.size() == 0) {
            return null;
        }
        return new String(buf.toByteArray(), "UTF-8");
    }

    private static void respond(Socket socket, Response response) throws IOException {
        OutputStream out = socket.getOutputStream();
        String head = "HTTP/1.1 " + response.status + " " + reason(response.status) + "\r\n"
                + "Content-Type: " + response.contentType + "; charset=utf-8\r\n"
                + "Content-Length: " + response.body.length + "\r\n"
                // Local tooling reads this from a file:// page or another origin.
                + "Access-Control-Allow-Origin: *\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes("UTF-8"));
        out.write(response.body);
        out.flush();
    }

    private static String reason(int status) {
        switch (status) {
            case 200: return "OK";
            case 400: return "Bad Request";
            case 404: return "Not Found";
            default:  return "Internal Server Error";
        }
    }

    /** Best-guess address on the local network, for logs and for the UI to show. */
    public static String localAddress() {
        try {
            for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                if (iface.isLoopback() || !iface.isUp()) {
                    continue;
                }
                for (InetAddress addr : Collections.list(iface.getInetAddresses())) {
                    String host = addr.getHostAddress();
                    if (host != null && host.indexOf(':') < 0) {
                        return host;
                    }
                }
            }
        } catch (Exception e) {
            // Fall through.
        }
        return "0.0.0.0";
    }
}
