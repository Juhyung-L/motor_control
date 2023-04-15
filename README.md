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
https://github.com/Juhyung-L/motor_control/blob/74b51d9d762a9252fd4e71dc08e287c9ea1a6531/main.c#L188-L208
if statement is placed in a timer interrupt service routine that is repeatedly called with a certain frequency. pot_data is the resistance reading from a potentiometer and its value is used to set the magnitude of motor velocity by setting the variable OC4RS, which controls the PWM signal duty cycle. So, the motor moves at a fixed speed and its direction is set to left or right if current_angle is not within 7.5 degrees of desired_angle.

# Proportional Controller
The second controller is the proportional controller. The block diagram of the controller is shown below.

![image](https://user-images.githubusercontent.com/102873080/232195599-76c0ed6b-1676-41b0-8801-fcac34222472.png)

This controller works by taking the difference between the current_angle and desired_angle, multiplying the difference by a constant named kp, then setting that value to be the velocity of the motor.
https://github.com/Juhyung-L/motor_control/blob/74b51d9d762a9252fd4e71dc08e287c9ea1a6531/main.c#L209-L215
The benefit of this controller compared to the angle-gap controller is that the velocity of the motor is high when current_angle is far away in value from desired_angle and low when the two angles are close in values.

# Proportional-Integral Controller
The last controller is the most complicated controller out of the three (relatively).

![image](https://user-images.githubusercontent.com/102873080/232196157-2df1eded-97cc-47f1-a6af-c059e11efd2e.png)

This controller is an extension of the proportional controller. The added feature is that the velocity of the motor is not only defined by the difference in current_angle and desired_angle. but also the integral of the difference.
https://github.com/Juhyung-L/motor_control/blob/74b51d9d762a9252fd4e71dc08e287c9ea1a6531/main.c#L216-L223
The integral is calculated by summing the difference in the angles. This method of calculating the integral is called the Euler approximation and it is a fairly good estimate of the integral because this block of code is called many times per second.

What makes this controller superior to the proportional controller is that the proportional controller cannot maintain a steady motor angle under constant force. Consider the following scenario: a constant force is applied to the wheel of the motor that shifts current_angle away from desired_angle. In this case, the proportional controller will start moving the motor only when the difference in the angles is non-zero. This means the motor will go back and forth between desired_angle and slightly less/more than desired_angle, causing and oscillatory motion. On the other hand, the proportional-integral controller will be able to maintain a steady angle even under constant force.

