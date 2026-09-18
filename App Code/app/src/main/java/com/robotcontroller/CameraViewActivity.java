package com.robotcontroller;

import android.os.Bundle;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;

public class CameraViewActivity extends AppCompatActivity {

    private MjpegStream mjpegStream;
    private String modeTitle;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera_view);

        ImageView cameraView = findViewById(R.id.camera_view);
        TextView tvStreamStatus = findViewById(R.id.tv_stream_status);
        TextView tvModeTitle = findViewById(R.id.tv_mode_title);
        TextView tvSubtitle = findViewById(R.id.tv_subtitle);

        findViewById(R.id.btn_back).setOnClickListener(v -> {
            BluetoothService.getInstance().sendStop();
            finish();
        });

        modeTitle = getIntent().getStringExtra("MODE_TITLE");
        String subtitle = getIntent().getStringExtra("SUBTITLE");

        if (modeTitle != null) {
            tvModeTitle.setText(modeTitle);
        }
        if (subtitle != null && !subtitle.isEmpty()) {
            tvSubtitle.setText(subtitle);
            tvSubtitle.setVisibility(View.VISIBLE);
        } else {
            tvSubtitle.setVisibility(View.GONE);
        }

        mjpegStream = new MjpegStream(cameraView);
        mjpegStream.setStatusListener(tvStreamStatus::setText);
        tvStreamStatus.setText("Connecting to camera...");
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (modeTitle != null) {
            switch (modeTitle) {
                case "NOISE":
                    BluetoothService.getInstance().sendEnterNoise();
                    break;
            }
        }
        mjpegStream.start();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mjpegStream.stop();
        BluetoothService.getInstance().sendStop();
    }
}