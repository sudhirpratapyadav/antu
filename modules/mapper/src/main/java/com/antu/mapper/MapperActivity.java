package com.antu.mapper;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.WindowManager;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

/**
 * The map, on the phone that is building it.
 *
 * <p>The console the ops node serves is the whole user interface — video, 2D
 * map, 3D map — and it runs in any browser. Here the browser is a WebView
 * pointed at the phone's own server, opened straight onto the 3D map. No
 * second UI to maintain: whatever the console can show on a laptop it shows
 * here, and a laptop on the same network sees the same page.
 *
 * <p>The server comes up a moment after the service does, so the first load
 * usually fails; the client retries until it answers.
 */
public final class MapperActivity extends Activity {

    private static final String CONSOLE = "http://127.0.0.1:8080/#view=map3d";
    private static final int RETRY_MS = 1500;

    private static java.lang.ref.WeakReference<MapperActivity> instance;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private WebView web;

    static void quit() {
        MapperActivity a = instance == null ? null : instance.get();
        if (a != null) {
            a.ui.post(a::finishAndRemoveTask);
        }
    }

    @Override protected void onCreate(Bundle state) {
        super.onCreate(state);
        instance = new java.lang.ref.WeakReference<>(this);
        // ARCore stops delivering frames the moment the display sleeps.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        web = new WebView(this);
        web.setBackgroundColor(Color.parseColor("#0b0e13"));
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);          // the console remembers its view
        s.setMediaPlaybackRequiresUserGesture(false);
        web.setWebViewClient(new WebViewClient() {
            @Override public void onReceivedError(WebView view, WebResourceRequest request,
                                                  WebResourceError error) {
                if (request.isForMainFrame()) {
                    ui.postDelayed(() -> view.loadUrl(CONSOLE), RETRY_MS);
                }
            }
        });
        setContentView(web);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M
                && checkSelfPermission(android.Manifest.permission.CAMERA)
                    != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[] {android.Manifest.permission.CAMERA}, 1);
        }

        Intent service = new Intent(this, MapperService.class);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(service);
        } else {
            startService(service);
        }
        ui.postDelayed(() -> web.loadUrl(CONSOLE), RETRY_MS);
    }

    @Override public void onRequestPermissionsResult(int code, String[] permissions,
                                                     int[] results) {
        super.onRequestPermissionsResult(code, permissions, results);
        // ARCore was created before the grant; the service restarts it cleanly.
        if (results.length > 0
                && results[0] == android.content.pm.PackageManager.PERMISSION_GRANTED
                && MapperService.graph() == null) {
            stopService(new Intent(this, MapperService.class));
            ui.postDelayed(() -> {
                Intent service = new Intent(this, MapperService.class);
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    startForegroundService(service);
                } else {
                    startService(service);
                }
            }, 500);
        }
    }

    @Override protected void onDestroy() {
        if (instance != null && instance.get() == this) {
            instance = null;
        }
        if (web != null) {
            web.destroy();
        }
        super.onDestroy();
    }
}
