within mbsSetupOptimizer;
model Vehicle
  import Modelica.Units.SI.*;
  
  parameter Acceleration ax = 0; // might stay constant but could change...
  Acceleration ay(start=0);
  parameter Velocity u = 15; // lon velocity, this is going to stay constant
  Velocity v(start=0);
  AngularVelocity yaw_rate(start=0);
  Angle delta_steering_wheel(start=0); // this changes with time
  
  
  Force X(start=0); // sum of longitudinal forces
  Force Y(start=0); // sum of lat forces
  Torque N(start=0); // yaw moment
  Angle r(start=0); // yaw angle
  
  Force X1(start=0); // front lon forces
  Force X2(start=0);
  Force Xa(start=0); // aero forces
  
  Force Y1(start=0); // front lateral forces
  Force Y2(start=0); // rear lateral forces
  
  Force delta_X1(start=0);
  Force delta_X2(start=0);
  
  Torque Nx(start=0);
  Torque Ny(start=0);
  
  // 11 = fl, 12 = fr, 21 = rl, 22 = rr
  Wheel wheel_fl(pos=1); // fy, fx, delta
  Wheel wheel_fr(pos=2);
  Wheel wheel_rl(pos=3);
  Wheel wheel_rr(pos=4);
  
  
equation
  delta_steering_wheel = 0.05*time; // in radians
  // equilibrium equations
  // VehicleParameters.vehicleMass*ax = X; // commented out for now since we are fixing ax
  VehicleParameters.vehicleMass*ay = Y;
  VehicleParameters.Izz*yaw_rate = N;
  yaw_rate = der(r);
  
  Xa = 0.5*VehicleParameters.rho_air*(u^2)*VehicleParameters.Cd*VehicleParameters.A;
  
  // ax = der(u) - v*r;
  ay = der(v) + u*r;
  
  Ny = Y1*VehicleParameters.a - Y2*VehicleParameters.b;
  Nx = delta_X1*VehicleParameters.trackwidth + delta_X2*VehicleParameters.trackwidth;
  
  X1 = -wheel_fl.Fy*sin(wheel_fl.delta) - wheel_fr.Fy*sin(wheel_fr.delta);
  X2 = wheel_rl.Fx + wheel_fr.Fx;
  Y1 = wheel_fl.Fy*cos(wheel_fl.delta) + wheel_fr.Fy*cos(wheel_fr.delta);
  Y2 = wheel_rl.Fy + wheel_rr.Fy;
  
  delta_X1 = 0.5*(wheel_fl.Fy*sin(wheel_fl.delta) - wheel_fr.Fy*sin(wheel_fr.delta));
  delta_X2 = 0.5*(wheel_rr.Fx - wheel_rl.Fx);
  
  X = X1 + X2 - Xa;
  Y = Y1 + Y2;
  
  N = Ny + Nx;
  
  // inputs to Wheel.mo
  wheel_fl.vx_body = u;
  wheel_fl.vy_body = v;
  wheel_fl.yaw_rate = yaw_rate;
  wheel_fl.ax = ax;
  wheel_fl.ay = ay;
  
  wheel_fr.vx_body = u;
  wheel_fr.vy_body = v;
  wheel_fr.yaw_rate = yaw_rate;
  wheel_fr.ax = ax;
  wheel_fr.ay = ay;
  
  wheel_rl.vx_body = u;
  wheel_rl.vy_body = v;
  wheel_rl.yaw_rate = yaw_rate;
  wheel_rl.ax = ax;
  wheel_rl.ay = ay;
  
  wheel_rr.vx_body = u;
  wheel_rr.vy_body = v;
  wheel_rr.yaw_rate = yaw_rate;
  wheel_rr.ax = ax;
  wheel_rr.ay = ay;
  
  wheel_fl.delta_steering_wheel = delta_steering_wheel;
  wheel_fr.delta_steering_wheel = delta_steering_wheel;
  wheel_rl.delta_steering_wheel = 0;
  wheel_rr.delta_steering_wheel = 0;
  


end Vehicle;
