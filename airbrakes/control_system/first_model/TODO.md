![alt text](https://github.com/t-whiteley/ucl_rocket/tree/master/airbrakes/control_system/first_model/block_diagram.jpg?raw=True)

TODO
* by putting real params, estimate change in area needed (for CAD) and torque needed (component selection)
* can only model signal to servo when geometry (CAD) is known, add saturation block to control
* add paramter angle of attack
* improve thrust curve
* lookup tables to reduce computation?? FEA and CFD??
* add kalman filter

notes on my implementation
* i'm assuming constant cd in the simulation however this is only to generate data for acceleration, velocity, and displacement, my algorithm is adaptive
* model is fragile, for some reason values in memory are huge, need to look closer at why

so whats going on
* apart from the controller in the main loop, most variables in model.py are trying to simluate the data that the algorithm would receive
* data is fed into predict_apogee(), this is a file which uses 4th order runge-kutte numerical differential equation solver to predict the current apogee in real time before achieving it
* back in model.py, the difference in desired vs current (predicted) apogee is the input to a proportional controller
* the controller changes the cross sectional area of the rocket, as a mechanical airbrake would, the effects of the servo reaction are not yet implemented

sources
* https://www.apogeerockets.com/Peak-of-Flight/Newsletter599
* https://www.rocketryforum.com/threads/active-altitude-control.174855/
* https://www.reddit.com/r/ControlTheory/comments/sliz1c/transfer_function_for_rocket_airbrake_pid_control/
* https://www.reddit.com/r/AerospaceEngineering/comments/slj0lm/transfer_function_for_rocket_airbrake_pid_control/
* https://www.rocketryforum.com/threads/active-altitude-control.174855/
* https://www.reddit.com/r/rocketry/comments/rsfw64/predicting_rocket_trajectory_using_rk4_integration/
* https://github.com/joshua-koehler/airbrakes/blob/master/airbrakes.ino
* https://offroad.engineering.queensu.ca/wp-content/uploads/2020/05/MECH-460-Final-Report-Team-03.pdf (good programming block diagram)
