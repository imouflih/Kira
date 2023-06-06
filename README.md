# Kira

Kira represents our state-of-the-art robotics solution that competed in the 2023 France Robotic Cup, helping us achieve a commendable 5th place finish. Developed in C++, it is implemented on a Raspberry Pi 4 and communicates with an Arduino Nano through the I2C interface. Kira is designed to efficiently manage all of the robot's movements and actions.

## Remote Connection to Raspberry Pi Using Visual Studio Code

1. First, ensure that the `Remote - SSH` extension is installed in your Visual Studio Code environment.
2. Open Visual Studio Code, press `F1` to open the command palette, type `Remote-SSH: Connect to Host...`, and hit `Enter`.
3. When prompted, enter the Raspberry Pi's IP address `pi@172.24.1.1` and press Enter.
4. Next, input the password for authentication. The password is noted in the blackboard.

## Accessing the Project Directory

Upon successful connection, open the integrated terminal in Visual Studio Code and navigate to the project directory with the following command:
``` cd Documents/Kira/src/cpp ```.

## Uploading Arduino Code

If there have been modifications in the Arduino's code, upload these changes by running:
``` ./Kira -u ```.
Note that the arduino code should be uploaded only if you change something in the arduino's code.

## Starting the Robot

To bring the robot to life, use:
``` ./Kira ```.

## Stopping the Robot

In order to halt the robot, issue:
``` ./Kira -s ```.

## Monitoring the Arduino Output

To inspect the Arduino's serial port output, utilize the following command:
``` sudo screen /dev/ttyUSB0 9600 ```.

## Precautionary Measures

Before you attempt to upload the Arduino's code, it's essential to ensure the /dev/tty/USB0 port isn't being used. This can be checked by executing:
``` sudo fuser /dev/ttyUSB0 ```.
If the port is occupied, terminate the process using:
``` sudo kill <pid> ```.
Replace `<pid>` with the relevant process identifier, which can be obtained using the ``sudo fuser /dev/ttyUSB0`` command.

## Frequently encountered problem

After each restart of the robot, it happens that the ports `/dev/ttyUSB` of the arduino and the lidar are reversed, to resolve this, simply change the file `LidarController.cpp` the port number, either `/dev/ttyUSB0` or `/dev/ttyUSB1`, and to change if necessary to upload the arduino code, the variable named `ARDUINO_PORT` in the file `Main.cpp`

## For Assistance

If you encounter any roadblocks or have queries, do not hesitate to contact me, Iliasse MOUFLIH, on any of my social media platforms.
