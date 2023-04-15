# motor_control
This code was written for a PIC32MX795F512L microprocessor on an Explorer 16/32 Board on the MPLAB X IDE. Because the code only works on the specified hardware, the focus of this repository is to show the method in which the DC motor was controlled, rather than the actual code itself.

The setup is shown below.
![image](https://user-images.githubusercontent.com/102873080/232191986-107e832b-d7ba-4396-a0c3-46192310355f.png)

There were 3 different types of motor controllers that were implemented
1. Angle-Gap Controller
2. Proportional controller
3. Proportional-Integral controller

# Angle-Gap Controller
This controller is the simplest method of controlling the motor. Essentially, the motor is set to turn right if its current angle exceeds the desired angle and turn left if it is below the desired angle.
