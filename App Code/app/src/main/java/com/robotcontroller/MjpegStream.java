package com.robotcontroller;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.ImageView;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

public class MjpegStream {

    private static final String TAG = "MjpegStream";
    private static final String STREAM_URL = "http://192.168.4.1/stream";

    private final ImageView targetView;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());

    private volatile boolean isStreaming = false;
    private Thread streamThread;
    private StatusListener statusListener;

    public interface StatusListener {
        void onStatusUpdate(String message);
    }

    public MjpegStream(ImageView targetView) {
        this.targetView = targetView;
    }

    public void setStatusListener(StatusListener listener) {
        this.statusListener = listener;
    }

    public void start() {
        if (isStreaming) return;
        isStreaming = true;

        streamThread = new Thread(() -> {
            HttpURLConnection connection = null;
            InputStream inputStream = null;

            try {
                URL url = new URL(STREAM_URL);
                connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(5000);
                connection.setReadTimeout(10000);
                connection.setRequestMethod("GET");
                connection.connect();

                if (connection.getResponseCode() != 200) {
                    updateStatus("HTTP error: " + connection.getResponseCode());
                    return;
                }

                updateStatus("Stream connected");
                inputStream = connection.getInputStream();
                readMjpegStream(inputStream);

            } catch (Exception e) {
                Log.e(TAG, "Stream error: " + e.getMessage());
                updateStatus("Connection failed — is Wi-Fi 'RobotCAM' connected?");
            } finally {
                try { if (inputStream != null) inputStream.close(); } catch (Exception ignored) {}
                if (connection != null) connection.disconnect();
            }
        }, "MJPEG-Stream-Thread");

        streamThread.setDaemon(true);
        streamThread.start();
    }

    public void stop() {
        isStreaming = false;
        if (streamThread != null) {
            streamThread.interrupt();
            streamThread = null;
        }
    }

    private void readMjpegStream(InputStream stream) throws Exception {
        byte[] buffer = new byte[4096];
        ByteArrayOutputStream frameBuffer = new ByteArrayOutputStream();
        boolean inFrame = false;

        while (isStreaming && !Thread.interrupted()) {
            int bytesRead = stream.read(buffer);
            if (bytesRead < 0) break;

            for (int i = 0; i < bytesRead; i++) {
                if (!inFrame && i + 1 < bytesRead
                        && (buffer[i] & 0xFF) == 0xFF
                        && (buffer[i + 1] & 0xFF) == 0xD8) {
                    frameBuffer.reset();
                    inFrame = true;
                }

                if (inFrame) {
                    frameBuffer.write(buffer[i]);

                    if (frameBuffer.size() >= 2) {
                        byte[] data = frameBuffer.toByteArray();
                        int len = data.length;
                        if ((data[len - 1] & 0xFF) == 0xD9
                                && (data[len - 2] & 0xFF) == 0xFF) {
                            inFrame = false;
                            displayFrame(data);
                        }
                    }
                }
            }
        }
    }

    private void displayFrame(byte[] jpegData) {
        final Bitmap bitmap = BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length);
        if (bitmap != null) {
            uiHandler.post(() -> targetView.setImageBitmap(bitmap));
        }
    }

    private void updateStatus(String message) {
        if (statusListener != null) {
            uiHandler.post(() -> statusListener.onStatusUpdate(message));
        }
    }
}