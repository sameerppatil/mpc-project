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

Since the problem we are solving here is optimization one, the number of inputs will have important impact on optimizatin solution. Also, dt, timestamp duration is related to

