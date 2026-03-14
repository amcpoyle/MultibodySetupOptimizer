within mbsSetupOptimizer;
model Vehicle
  import Modelica.Units.SI.*;
  
  parameter Radius R = 7.625; // m, corner radius
  Velocity V = 15;
  Angle delta_wheel; // steering wheel angle
  Angle delta(start=0); // steer angle of the front tires
  
  AngularVelocity r_dot(start=0); // yaw rate
  Acceleration ax = 0;
  Acceleration ay(start=0);
  Angle beta(start=0); // vehicle side slip angle
  
  Velocity vx(start=0);
  Velocity vy(start=0);
  
  // create an instance of each Wheel
  Wheel wheel_fl(pos = 1);
  Wheel wheel_fr(pos = 2);
  Wheel wheel_rl(pos = 3);
  Wheel wheel_rr(pos = 4);
  
  // wheel forces - can just ref from Wheel.mo directly
  // Force fx_fl, fy_fl, fz_fl;
  // Force fx_fr, fy_fr, fz_fr;
  // Force fx_rl, fy_rl, fz_rl;
  // Force fx_rr, fy_rr, fz_rr;
  
  // input connectors

  
  // output connectors
  // mbsSetupOptimizer.connectors.AngleOutput beta_output;
  // mbsSetupOptimizer.connectors.AngleOutput delta_output;
  
equation
  delta_wheel = 0.5*(time/5.0); // ramp from 0 to 0.25 radians over 5 seconds
  r_dot = V / R;
  ay = (V^2)/R;
  delta = delta_wheel/VehicleParameters.steer_ratio;
  beta = atan((VehicleParameters.b*tan(delta))/(VehicleParameters.a + VehicleParameters.b));
  
  vx = V*cos(beta);
  vy = V*sin(beta);
  
  // info transfer from Vehicle.mo to Wheel.mo
  wheel_fl.v_body.vx = vx;
  wheel_fl.v_body.vy = vy;
  wheel_fr.v_body.vx = vx;
  wheel_fr.v_body.vy = vy;
  wheel_rl.v_body.vx = vx;
  wheel_rl.v_body.vy = vy;
  wheel_rr.v_body.vx = vx;
  wheel_rr.v_body.vy = vy;
  
  wheel_fl.accel_input.ax = ax;
  wheel_fl.accel_input.ay = ay;
  wheel_fr.accel_input.ax = ax;
  wheel_fr.accel_input.ay = ay;
  wheel_rl.accel_input.ax = ax;
  wheel_rl.accel_input.ay = ay;
  wheel_rr.accel_input.ax = ax;
  wheel_rr.accel_input.ay = ay;
  
  wheel_fl.yaw_rate = r_dot;
  wheel_fr.yaw_rate = r_dot;
  wheel_rl.yaw_rate = r_dot;
  wheel_rr.yaw_rate = r_dot;
  
  wheel_fl.delta = delta;
  wheel_fr.delta = delta;
  wheel_rl.delta = delta;
  wheel_rr.delta = delta;


end Vehicle;
