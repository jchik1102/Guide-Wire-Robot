package com.robotcontroller;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.content.Intent;

public class MainActivity extends AppCompatActivity {

    private static final int PERMISSION_REQUEST_CODE = 1001;
    private TextView tvStatus;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        tvStatus = findViewById(R.id.tv_status);

        findViewById(R.id.btn_guide_wire_mode).setOnClickListener(v -> {
            startActivity(new Intent(this, GuideWirePathActivity.class));
        });

        findViewById(R.id.btn_manual_mode).setOnClickListener(v -> {
            startActivity(new Intent(this, ManualModeActivity.class));
        });

        findViewById(R.id.btn_learning_mode).setOnClickListener(v -> {
            startActivity(new Intent(this, LearningModeActivity.class));
        });

        findViewById(R.id.btn_noise_mode).setOnClickListener(v -> {
            Intent intent = new Intent(this, CameraViewActivity.class);
            intent.putExtra("MODE_TITLE", "NOISE");
            startActivity(intent);
        });

        checkPermissionsAndConnect();
    }

    @Override
    protected void onResume() {
        super.onResume();
        updateStatusText();
    }

    private void checkPermissionsAndConnect() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this,
                        new String[]{
                                Manifest.permission.BLUETOOTH_CONNECT,
                                Manifest.permission.BLUETOOTH_SCAN
                        },
                        PERMISSION_REQUEST_CODE);
                return;
            }
        } else {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                        PERMISSION_REQUEST_CODE);
                return;
            }
        }

        connectBluetooth();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                connectBluetooth();
            } else {
                tvStatus.setText("Bluetooth permission denied");
                Toast.makeText(this, "Bluetooth permission is required", Toast.LENGTH_LONG).show();
            }
        }
    }

    private void connectBluetooth() {
        tvStatus.setText("Connecting to HC-06...");

        BluetoothService bt = BluetoothService.getInstance();
        bt.setConnectionListener((status, connected) -> {
            uiHandler.post(() -> tvStatus.setText(status));
        });

        new Thread(() -> {
            bt.connect(this);
        }).start();
    }

    private void updateStatusText() {
        BluetoothService bt = BluetoothService.getInstance();
        if (bt.isConnected()) {
            tvStatus.setText("Connected to HC-06");
        } else {
            tvStatus.setText("Not connected");
        }
    }
}