# Report on Model-Predictive-Control project

#### Objective
This aim of this project is to discuss design decisions that went in while implementing Model Predictive Control project. It also talks about how system factors such as latency affect behavior of model.

### Model

The aim of the project is to calculate acceleration and steering values based on error optimization in difference between reference trajectory and predicted trajectory. We are working on global kinematic model rather than dynamic model which deals with effect of gravity and mass.

The model can be described mathematically as follows:

x~t+1~ = x~t~ + v~t~ * cos(ψ~t~) * dt
y~t+1~ = y~t~ + v~t~ * sin(ψ~t~) * dt
ψ~t+1~ = ψ~t~ + v~t~/L~f~ * δ ∗ dt
v~t+1~ = v~t~ + a~t~ * dt

where
- x, y are coordinates of vehicle
- v is the car velocity
- dt is time interval
- δ is the steering delta
- ψ is the vehicle orientation w.r.t to X-axis
- a is acceleration
- L~f~ is distance between front of vehicle and center of gravity of vehicle.

##### Cross track error

This error represents the error between vehicle's position relative of vehicle's position. The model also optimizes this error.

cte~t+1~ = cte~t~ + v~t~ * sin(eψ~t~)*dt

##### Orientation error

This error represents between the difference orientation of reference trajectory and vehicle's actual orientation.

 eψ~t+1~ = eψ~t~ + v~t~/L~f~ * δ ∗ dt

### Timestamp length and dt

Since the problem we are solving here is optimization one, the number of inputs will have important impact on optimization solution. Also, dt, timestamp duration is directly affects the number of actuations.

I experimented with values of N as 10, 20, 30 and dt value of 0.05. I observed, longer the N value, longer is look forward. So for values higher than 25, I observed that cars veers a lot on straight road at short distances. The value of N has to be a function of top speed and radius of turns on track. A good test for this exercise would be simulating the run on F1 track having high speed turns.

I finally settled for value of N as 20 and dt as 0.05 since these valued seemed like perfect for given track. But in real world, I believe these factors would have to be dynamically modified and can be preloaded from map since maps can store road turn curvature at each end.

### Preprocessing Waypoints

Since we are given map coordinates, we pre-process the waypoints to transform them into vehicle's coordinate system with vehicle at x,y = 0,0. Then I fitted the polynomial with 2nd order polynomial to predict the trajectory.

### Latency

Since the MPC computation are compute intensive and physical actuators would their own delay, we need to account for this latency in prediction. From classroom lessons, we assumed the latency to be of 100ms and adjusted the value of cross track error and orientation error based on latency in main.cpp between line number 158-160.
