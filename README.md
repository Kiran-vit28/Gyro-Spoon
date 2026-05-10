# Gyro-Spoon
A prototype spoon designed to help people with Essential tremor or Parkinson's disease eat more comfortably.
Hardware used :
          - ESP WROOM 32 Microcontroller
          - MPU6050 ( Gyroscopic sensor )
          - Micro Servos ( SG90 ) 

The system begins with the MPU (IMU Sensor), which continuously detects hand movement and tremor data such as acceleration and angular rotation. This sensor sends the collected motion data to the ESP32 microcontroller through I2C communication.

Inside the ESP32, the sensor readings are processed using sensor fusion algorithms to obtain accurate orientation and motion information. The ESP32 then applies the concept of a 2-axis PID (Proportional–Integral–Derivative) controller. The PID controller continuously calculates the error between the unwanted hand movement and the desired stable spoon position. Based on this error, it generates corrective control signals to counteract the tremors in real time.

These control signals are sent as PWM (Pulse Width Modulation) outputs to the servo drivers, which control two servo motors:

Servo X controls the pitch movement
Servo Y controls the roll movement

Finally, the two servos operate the 2-axis gimbal mechanism attached to the spoon head. The gimbal stabilizes the spoon by moving in the opposite direction of the detected tremors, helping keep the spoon level and steady while the user is eating.
