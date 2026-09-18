package com.robotcontroller;

import android.os.Bundle;
import android.os.CountDownTimer;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;

public class LearningModeActivity extends AppCompatActivity {

    private MjpegStream mjpegStream;
    private CountDownTimer countDownTimer;
    private char lastCommand = 'S';

    private View readyOverlay;
    private View joystickContainer;
    private JoystickView joystickView;
    private TextView tvJoystickInfo;
    private TextView tvTimer;
    private View btnReplay;
    private ImageView cameraView;
    private TextView tvStreamStatus;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_learning_mode);

        readyOverlay      = findViewById(R.id.ready_overlay);
        joystickContainer = findViewById(R.id.joystick_container);
        joystickView      = findViewById(R.id.joystick_view);
        tvJoystickInfo    = findViewById(R.id.tv_joystick_info);
        tvTimer           = findViewById(R.id.tv_timer);
        btnReplay         = findViewById(R.id.btn_replay);
        cameraView        = findViewById(R.id.camera_view);
        tvStreamStatus    = findViewById(R.id.tv_stream_status);

        findViewById(R.id.btn_back).setOnClickListener(v -> {
            if (countDownTimer != null) countDownTimer.cancel();
            BluetoothService.getInstance().sendStop();
            finish();
        });

        findViewById(R.id.btn_back_ready).setOnClickListener(v -> {
            BluetoothService.getInstance().sendStop();
            finish();
        });

        mjpegStream = new MjpegStream(cameraView);
        mjpegStream.setStatusListener(tvStreamStatus::setText);

        showReadyState();

        findViewById(R.id.btn_start_joystick).setOnClickListener(v -> {
            BluetoothService.getInstance().sendStartJoystickRecord();
            showRecordingState();
        });

        findViewById(R.id.btn_start_sensor).setOnClickListener(v -> {
            BluetoothService.getInstance().sendStartSensorRecord();
            showRecordingState();
        });

        joystickView.setOnJoystickMoveListener((xPercent, yPercent, angle, strength) -> {
            String info = String.format("X: %d%%  Y: %d%%", xPercent, yPercent);
            tvJoystickInfo.setText(info);

            char cmd = joystickToCommand(xPercent, yPercent, strength);
            if (cmd != lastCommand) {
                lastCommand = cmd;
                BluetoothService.getInstance().send(cmd);
            }
        });

        btnReplay.setOnClickListener(v -> {
            BluetoothService.getInstance().sendReplay();
            showReadyState();
        });
    }

    private char joystickToCommand(int xPercent, int yPercent, int strength) {
        if (strength < 25) {
            return 'S';
        }

        int absX = Math.abs(xPercent);
        int absY = Math.abs(yPercent);

        if (absY >= absX) {
            return (yPercent < 0) ? 'F' : 'B';
        } else {
            return (xPercent > 0) ? 'R' : 'L';
        }
    }

    private void showReadyState() {
        readyOverlay.setVisibility(View.VISIBLE);
        joystickContainer.setVisibility(View.GONE);
        tvTimer.setVisibility(View.GONE);
        btnReplay.setVisibility(View.GONE);
        tvStreamStatus.setText("Waiting to start...");
    }

    private void showRecordingState() {
        readyOverlay.setVisibility(View.GONE);
        joystickContainer.setVisibility(View.VISIBLE);
        tvTimer.setVisibility(View.VISIBLE);
        btnReplay.setVisibility(View.GONE);

        mjpegStream.start();

        countDownTimer = new CountDownTimer(20000, 1000) {
            @Override
            public void onTick(long millisUntilFinished) {
                int secondsLeft = (int) (millisUntilFinished / 1000);
                tvTimer.setText(String.format("%02d:%02d", secondsLeft / 60, secondsLeft % 60));
            }

            @Override
            public void onFinish() {
                tvTimer.setText("00:00");
                BluetoothService.getInstance().sendStop();
                lastCommand = 'S';
                showReplayState();
            }
        }.start();
    }

    private void showReplayState() {
        joystickContainer.setVisibility(View.GONE);
        tvTimer.setVisibility(View.GONE);
        btnReplay.setVisibility(View.VISIBLE);
    }

    @Override
    protected void onResume() {
        super.onResume();
        BluetoothService.getInstance().sendEnterLearning();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mjpegStream.stop();
        if (countDownTimer != null) countDownTimer.cancel();
        BluetoothService.getInstance().sendStop();
        lastCommand = 'S';
    }
}