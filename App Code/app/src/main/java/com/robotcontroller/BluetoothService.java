package com.robotcontroller;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.util.Log;

import androidx.core.content.ContextCompat;

import java.io.IOException;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class BluetoothService {

    private static final String TAG = "BluetoothService";
    private static final String HC06_NAME = "HC-06";
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static BluetoothService instance;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothSocket socket;
    private OutputStream outputStream;
    private boolean isConnected = false;

    private ConnectionListener connectionListener;

    public interface ConnectionListener {
        void onConnectionUpdate(String status, boolean connected);
    }

    private BluetoothService() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    }

    public static synchronized BluetoothService getInstance() {
        if (instance == null) {
            instance = new BluetoothService();
        }
        return instance;
    }

    public void setConnectionListener(ConnectionListener listener) {
        this.connectionListener = listener;
    }

    public boolean isConnected() {
        return isConnected;
    }

    public void connect(Context context) {
        if (isConnected) {
            notifyStatus("Already connected", true);
            return;
        }

        if (bluetoothAdapter == null) {
            notifyStatus("Bluetooth not supported", false);
            return;
        }

        if (!bluetoothAdapter.isEnabled()) {
            notifyStatus("Bluetooth is off — turn it on in Settings", false);
            return;
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                notifyStatus("Bluetooth permission not granted", false);
                return;
            }
        }

        BluetoothDevice hc06 = findHC06(context);
        if (hc06 == null) {
            notifyStatus("HC-06 not found — pair it in Settings first", false);
            return;
        }

        try {
            notifyStatus("Connecting to HC-06...", false);

            socket = hc06.createRfcommSocketToServiceRecord(SPP_UUID);
            bluetoothAdapter.cancelDiscovery();
            socket.connect();

            outputStream = socket.getOutputStream();
            isConnected = true;

            notifyStatus("Connected to HC-06", true);
            Log.i(TAG, "Connected to HC-06 successfully");

        } catch (IOException e) {
            Log.e(TAG, "Connection failed: " + e.getMessage());
            notifyStatus("Connection failed — is HC-06 powered on?", false);
            disconnect();
        } catch (SecurityException e) {
            Log.e(TAG, "Permission error: " + e.getMessage());
            notifyStatus("Bluetooth permission denied", false);
        }
    }

    public void disconnect() {
        isConnected = false;
        try {
            if (outputStream != null) outputStream.close();
        } catch (IOException ignored) {}
        try {
            if (socket != null) socket.close();
        } catch (IOException ignored) {}
        outputStream = null;
        socket = null;
    }

    public void send(char command) {
        if (!isConnected || outputStream == null) {
            Log.w(TAG, "Not connected, can't send: " + command);
            return;
        }

        new Thread(() -> {
            try {
                outputStream.write(command);
                outputStream.flush();
                Log.d(TAG, "Sent: " + command);
            } catch (IOException e) {
                Log.e(TAG, "Send failed: " + e.getMessage());
                isConnected = false;
                notifyStatus("Connection lost", false);
            }
        }).start();
    }

    public void sendForward()  { send('F'); }
    public void sendBackward() { send('B'); }
    public void sendLeft()     { send('L'); }
    public void sendRight()    { send('R'); }
    public void sendStop()     { send('S'); }

    public void sendEnterManual()   { send('m'); }
    public void sendEnterNoise()    { send('N'); }
    public void sendEnterLearning() { send('E'); }

    public void sendGuidewirePath(int path) { send((char)('0' + path)); }

    public void sendStartJoystickRecord() { send('T'); }
    public void sendStartSensorRecord()   { send('U'); }
    public void sendReplay()              { send('P'); }

    private BluetoothDevice findHC06(Context context) {
        try {
            Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
            if (pairedDevices == null) return null;

            for (BluetoothDevice device : pairedDevices) {
                String name = device.getName();
                if (name != null && name.equals(HC06_NAME)) {
                    Log.i(TAG, "Found HC-06: " + device.getAddress());
                    return device;
                }
            }
        } catch (SecurityException e) {
            Log.e(TAG, "Permission error finding HC-06: " + e.getMessage());
        }

        return null;
    }

    private void notifyStatus(String status, boolean connected) {
        Log.i(TAG, "Status: " + status + " (connected=" + connected + ")");
        if (connectionListener != null) {
            connectionListener.onConnectionUpdate(status, connected);
        }
    }
}