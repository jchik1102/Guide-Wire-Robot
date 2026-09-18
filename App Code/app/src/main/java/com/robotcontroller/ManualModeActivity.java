package com.robotcontroller;

import android.os.Bundle;
import android.widget.ImageView;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;

public class ManualModeActivity extends AppCompatActivity {

    private MjpegStream mjpegStream;
    private char lastCommand = 'S';

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_manual_mode);

        ImageView cameraView = findViewById(R.id.camera_view);
        JoystickView joystickView = findViewById(R.id.joystick_view);
        TextView tvJoystickInfo = findViewById(R.id.tv_joystick_info);
        TextView tvStreamStatus = findViewById(R.id.tv_stream_status);

        findViewById(R.id.btn_back).setOnClickListener(v -> finish());

        mjpegStream = new MjpegStream(cameraView);
        mjpegStream.setStatusListener(tvStreamStatus::setText);
        tvStreamStatus.setText("Connecting to camera...");

        joystickView.setOnJoystickMoveListener((xPercent, yPercent, angle, strength) -> {
            String info = String.format("X: %d%%  Y: %d%%", xPercent, yPercent);
            tvJoystickInfo.setText(info);

            char cmd = joystickToCommand(xPercent, yPercent, strength);

            if (cmd != lastCommand) {
                lastCommand = cmd;
                BluetoothService.getInstance().send(cmd);
            }
        });
    }

    private char joystickToCommand(int xPercent, int yPercent, int strength) {
        if (strength < 25) {
            return 'S';
        }

        int absX = Math.abs(xPercent);
        int absY = Math.abs(yPercent);

        if (absY >= absX) {
            if (yPercent < 0) {
                return 'F';
            } else {
                return 'B';
            }
        } else {
            if (xPercent > 0) {
                return 'R';
            } else {
                return 'L';
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        BluetoothService.getInstance().sendEnterManual();
        mjpegStream.start();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mjpegStream.stop();
        BluetoothService.getInstance().sendStop();
        lastCommand = 'S';
    }
}