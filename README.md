# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Controller
The MPC used in this project was based on the Kinematic model. The state for the kinematic model include the car's coordinates, velocity(v), orientation(psi), error in orientation (epsi), and the cross-track error(cte). The model is updated using two actuators, the steering angle (delta), and the acceleration (throttle). 
The update equations used were as follows:
```C++
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 + v1 * delta0 / Lf * dt;
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
## Timestep Length and Elapsed Duration (N & dt)
I started off with a high N = 20 and a duration dt = 0.05, but this caused the car to behave erratically around the track. Afterwards, N was decreased which lead to a slightly better performance, but the car still struggled at sharp curves. Finally, setting N = 12 and dt = 0.1 (which is equal to the latency) caused the car to drive safely around the track.

## Polynomial Fitting and MPC Preprocessing
The waypoints' coordinates are transformed from the Map coordinate system to the car coordinate system using the following function:
```C++
// Coordinate Transformation from map to car
void transformToCarCoordinates(vector<double>& ptsx,vector<double>& ptsy, vector<double>& next_x_vals, vector<double>& next_y_vals, double px, double py, double psi){

  for(uint i = 0; i < ptsx.size(); i++){
    next_x_vals.push_back( (ptsx[i]-px) * cos(psi) + (ptsy[i]-py) * sin(psi));
    next_y_vals.push_back( (ptsy[i]-py) * cos(psi) - (ptsx[i]-px) * sin(psi));
  }
  
}
```
The transformed waypoints were then fitted using a third degree polynomial. The cofficients of that polynomial were then passed to the solver to calculate the optimum solution for the actuators' values.

## Model Predictive Control with Latency
I was a little confused on how to incorporate the latency into MPC, but after doing some digging and searching on the forum, I followed [James's suggestion](https://github.com/jamesjosephreynolds) by taking the initial state delayed by one latency unit as shown in the code snippet below:
```C++
double psi = j[1]["psi"];
double v = j[1]["speed"];

// add latency to initial state
px = v*LATENCY;
psi = -v * delta * LATENCY / Lf;

// calculate cte
double cte = polyeval(coeffs, px);

// calculate epsi
double epsi = atan(polydereval(coeffs, px));

// set up the state
state << px, 0.0, psi, v, cte, epsi;
```
