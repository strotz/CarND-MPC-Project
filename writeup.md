vehicle model has state and actuators
There are 6 values that describe state of kinematic model of the vehicle:
1. x position
2. y position
3. orientation
4. velocity
5. cross track error (CTE)
6. orientation error

Actuators:
1. steering angle
2. accelerator/break combination

The goal of MPC is to write application that will build and optimize prediction of the vehicle state. 
In order to archive it ipopt and cppad model describes how vehicle will look over N iterative steps. 

Steps that I took during this project:
1. clone repo
2. make sure that cmake and make works
3. implemented model of vehicle in MPC.cpp
- kinematic model (how vehicle state changes every step)
- constraints on actuators
- constraints on vehicle state
4. implemented simple cost function that depends on CTE, EPSI. Cost also include usage of actuators and reduce 
 change of actuators value over the time.
5. originally I removed 100 milliseconds latency, reduced target speed to 10 m/s
6. a couple adjustments made due to simulator:
- model adjusted because psi in simulator has same orientation as a steering wheel, i.e. clockwise
- conversion of speed (MPH) to velocity (meter/seconds) required
- steering angle has to be scaled from rad to [-1,1] range
7. 3rd order approximation of viewpoints implemented (originally was 2rd order, but fixed after review)
7. in order to achive stable 10 m/s rides next tuning were made:
- N/dt combination selected to be 10/0.1 (originally, I choose 10/0.12, but reset to 10/0.1 after review)
Large number of N introduced computation latency and position instability
- cost function adjusted to make emphisise steering angle derivative component:
  
  fg[0] += 400 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  
8. reference velocity was increased multipl times and cost function adjusted
  
9. Stable trajectory on simultor reached 65 MPH

10. Restored latency handling:
 - reset target velocity to 10 m/s
 - instead feeding initial state of the vehicle to optimizer, kinematic model were applied with 100 ms latency and that state used 
 - cost function were adjusted
 - reference velocity iteratively increased back to 30 m/s with required adjustments of cost function 
   
    fg[0] += 1500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);

   