#RobotArm
A robotic arm installed on drone for mechanical research.  

##Features
> * 8 servos are applied at its joints to simulate human's action;  
> * 0.1&deg; resolution;  
> * Up to 1000&deg;/s angular velocity;  
> * Real time response and schedule.  
> * Adjustable S-Curve, capable of asymmetrical settings or parameters.

##Platform
MCU: STM32F103C8(no FPU)  
RTOS: uC/OS-II  
IDE: IAR is suggested  
Servo: KST  

##Implementation
&emsp;The key part of this tiny project is to smooth the motion of each servo and to eliminate pause and shaking.  
&emsp;An effective solution is to program the motion of each servo corresponding to an S-Curve with 7 segments so that the ___angular acceleration___ is continous. As the S-curve is determined by the ratio of each segment, lots of papers and programs assume the acceleration and deceleration parts are symmetric so that the deduction is simplified.  
&emsp;However, I was not in favor of this practice. So I derived the key parameters when given the ratio of each segment. Although the floating point computation costs much more cpu cycles(almost 1us per plus/minus/times/divide operation) and there will be almost 500 times of such operations for each command, it's acceptable when the clock frequence is set to 72MHz.  
&emsp;Besides, the RTOS is used for receiving commands and dispatching tasks to corresponding servo. In this way, synchronization work were done automatically instead of buggy manual labour. 

##Decleration
Just for research.   
Suggestions are welcome.
