\# Guide-Wire Robot



An ELEC 291 robot project combining guide-wire navigation, handheld remote control, an Android control app, and a live camera view. The robot uses inductor readings to follow a wire and detect intersections. The repository also includes code for an IR remote, Bluetooth commands, and an ESP32-CAM video stream.



\## Project components



| Component | What it does |

| --- | --- |

| Robot controller | Reads guide-wire sensors and controls the motors. The robot code includes path selection and intersection-handling implementations. |

| Handheld remote | Contains EFM8-based remote code for joystick input, IR communication, LCD output, and feedback features. |

| Android app | Provides manual joystick control, three guide-wire path selections, a learning-mode interface, and a camera view. It sends commands over Bluetooth to a paired HC-06 module. |

| ESP32-CAM | Hosts a Wi-Fi network and serves an MJPEG video stream for the app’s camera view. |



\## Repository layout



\- \[Robot and Remote Code](Robot%20and%20Remote%20Code/) — robot and remote firmware, camera firmware, path configurations, and development versions.

\- \[App Code](App%20Code/) — Android Studio project for the robot controller app.



\## Control modes



\- \*\*Guide-wire paths:\*\* Select one of three paths in the app. The robot code contains predefined actions for intersections along each path.

\- \*\*Manual control:\*\* Use the app’s joystick to send forward, backward, left, right, and stop commands over Bluetooth.

\- \*\*Learning mode:\*\* The app provides joystick and sensor recording options, a 20-second recording screen, and a replay command.

\- \*\*Camera view:\*\* View the ESP32-CAM stream while using supported app screens. The app also includes a noise-mode screen.



The handheld remote provides another control interface through its own firmware and IR circuitry.



\## Running the Android app



1\. Open `App Code` as a project in Android Studio.

2\. Build and install the app on an Android device with Bluetooth.

3\. Pair the phone with the robot’s \*\*HC-06\*\* module in Android settings, then grant the app’s requested Bluetooth permissions.

4\. For the camera view, power the ESP32-CAM and connect the phone to its \*\*RobotCAM\*\* Wi-Fi network. The app expects the stream at `http://192.168.4.1/stream`.



The camera firmware is in `Robot and Remote Code/ESP32\_CAM\_Firmware.ino`. Hardware is required to use the robot controls and video stream.



\## Development notes



This repository preserves multiple robot and remote code versions from development. Files with names such as `wip`, `test`, and `failing` are experiments or earlier iterations; they should not be treated as interchangeable final firmware. Check the hardware pin mappings and the intended controller before compiling or flashing a particular file.

