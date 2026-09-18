package com.robotcontroller;

import android.content.Intent;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;

public class GuideWirePathActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_guide_wire_path);

        findViewById(R.id.btn_back).setOnClickListener(v -> finish());

        findViewById(R.id.btn_path_1).setOnClickListener(v -> {
            BluetoothService.getInstance().sendGuidewirePath(1);
            launchCameraWithPath("Path 1");
        });

        findViewById(R.id.btn_path_2).setOnClickListener(v -> {
            BluetoothService.getInstance().sendGuidewirePath(2);
            launchCameraWithPath("Path 2");
        });

        findViewById(R.id.btn_path_3).setOnClickListener(v -> {
            BluetoothService.getInstance().sendGuidewirePath(3);
            launchCameraWithPath("Path 3");
        });
    }

    private void launchCameraWithPath(String pathName) {
        Intent intent = new Intent(this, CameraViewActivity.class);
        intent.putExtra("MODE_TITLE", "GUIDE WIRE");
        intent.putExtra("SUBTITLE", pathName);
        startActivity(intent);
    }
}