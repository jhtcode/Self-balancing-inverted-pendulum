# Self-balancing-inverted-pendulum
self balancing an inverted pendulum with reaction wheel.
youtube vid: https://www.youtube.com/watch?v=ZYu1y-V240g 

The arduino code: invertedPendulum_final.ino

the pendulum control uses PID and PD controller. the PID controller input is the pendulum setpoint and the output is a signal that determines motor directions and voltage. The PD controller shifting the setpoint, which somehow help slowing down and stopping the reaction wheel.
![image](https://user-images.githubusercontent.com/91642218/150599567-e161b52b-4029-43aa-a62c-ab9141ffb10f.png)
