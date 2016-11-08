# RobotArm

##Description
A robotic arm installed on drone for purpose of mechanical research.  

##Feature
8 servos are applied at its joints to simulate various action;  
0.1&deg resolution;  
Angular velocity up to 1000&deg/s;  
Real time response and schedule.
Adjustable S-Curve capable of asymmetrical settings or parameters.

##Platform
MCU: STM32F103C8
RTOS: uC/OS-II
IDE: IAR is suggested
Servo: KST

##Implementation
The key part of this little project is to smooth the motion of each servo and eliminate pause or shake.  
An effective solution is to program the motion of each servo to a S-Curve with 7 segments so that the angular acceleration is continous.  
As the S-curve is clearly determined by the ratio of each segment, many existing papers or programs assume the acceleration and deceleration parts are symmetric so that the deduction is simplified.  
I'm not in favor of this practice. So I derivate the key parameters given the ratio of each segment.  
Although the floating point operation takes much computation for each movement, it's acceptable when the clock freq is 72MHz.  
Besides, the rtos is used for receiving commands and do some synchronization work.  

##Decleration
Just for research. Suggestions are welcome.