# UR Project

The aim of this project is to analyse the _Elastic Structure Preserving_ (ESP) control technique that helps in assigning a damped behaviour to robot manipulators with elastic transmissions in order to avoid the typical oscillatory behaviours.

## Simulations

The simulatons have been done in Matlab/Simulink and the control technique has been implemented on a 3R planar robot with nonlinear elastic transmissions which lays on a vertical plane.
Moreover, we have considered the linear elastic trasmission case in which we have assigned also a desired stifness in order ot be able to shape properly the inertia of the robot in the closed-loop behaviour. 
In all the simulations the control techiniques have been compared to a simple PD law with gravity compensation in order to highlight the capability of the control of avoiding oscillatory behaviours. 

## Code-Matlab

1. The _main.m_ regards the implementation and comparison of the ESP and ESP+ control techinques in the nonlinear elastic transmission case.
2. The _main_linear.m_ regards the implementation and comparison of the ESP and ESP+ control techinques in the linear elastic transmission case.

## Code-Simulink

1. _planar_3R_u_ESP.slx_, _planar_3R_u_ESPp.slx_ and _planar_3R_u_PD.slx_ are the block schemes regarding the nonlinear elastic transmission case.
1. _ESP_linear_stiff.slx_ and _ESPp_linear_stiff.slx_ are the block schemes regarding the linear elastic transmission case.
