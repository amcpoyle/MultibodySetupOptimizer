within mbsSetupOptimizer;
model Vehicle
  import Modelica.Units.SI.*;
  
  parameter Radius R = 7.625; // m, corner radius
  Velocity V = 15;
  Angle delta_wheel = 0.15; // steering wheel angle
  Angle delta(start=0); // steer angle of the front tires
  
  AngularVelocity r_dot(start=0); // yaw rate
  Acceleration ax = 0;
  Acceleration ay(start=0);
  Angle beta(start=0); // vehicle side slip angle
  
  Velocity vx(start=0);
  Velocity vy(start=0);
  
  // create an instance of each Wheel
  Wheel wheel_fl(pos = Wheel.WheelPosition.FL);
  Wheel wheel_fr(pos = Wheel.WheelPosition.FR);
  Wheel wheel_rl(pos = Wheel.WheelPosition.RL);
  Wheel wheel_rr(pos = Wheel.WheelPosition.RR);
  
  // wheel forces
  Force fx_fl, fy_fl, fz_fl;
  Force fx_fr, fy_fr, fz_fr;
  Force fx_rl, fy_rl, fz_rl;
  Force fx_rr, fy_rr, fz_rr;
  
  // input connectors

  
  // output connectors
  mbsSetupOptimizer.connectors.AngleOutput beta_output;
  mbsSetupOptimizer.connectors.AngleOutput delta_output;
  
equation
  r_dot = V / R;
  ay = (V^2)/R;
  delta = delta_wheel/VehicleParameters.steer_ratio;
  beta = atan((VehicleParameters.b*tan(delta))/(VehicleParameters.a + VehicleParameters.b));
  
  vx = V*cos(beta);
  vy = V*sin(beta);
  
  wheel_fl.v_body.vx = vx;
  wheel_fl.v_body.vy = vy;
  
  

end Vehicle;
