within mbsSetupOptimizer;
model Wheel
  import Modelica.Units.SI.*;
  
  type WheelPosition = enumeration(FL, FR, RL, RR);
  parameter WheelPosition pos;
  
  Velocity vx_wheel(start=0); // lon velocity
  Velocity vy_wheel(start=0); // lat velocity
  Velocity vx_body(start=0);
  Velocity vy_body(start=0);
  AngularVelocity yaw_rate(start=0);
  Angle delta(start=0);
  
  // tire forces
  Force Fx;
  Force Fy;
  Force Fz;
  
  Angle alpha; // slip angle
  Real kappa; // lon slip
  
  Acceleration ax(start=0);
  Acceleration ay(start=0);
  
  // input connectors
  // we need steer angle of the wheel, body u and v, yaw rate
  mbsSetupOptimizer.connectors.AngleInput delta_input;
  mbsSetupOptimizer.connectors.VelocityInput v_body;
  mbsSetupOptimizer.connectors.AngularVelocityInput yaw_rate_input;
  mbsSetupOptimizer.connectors.AccelerationInput accel_input;
  
equation
  if pos == WheelPosition.FL then
    vy_wheel = vy_body + yaw_rate*VehicleParameters.a;
    vx_wheel = vx_body - yaw_rate*(VehicleParameters.trackwidth/2);
    Fz = NormalLoads(ax, ay, vx_wheel, "FL");
  elseif pos == WheelPosition.FR then
    vy_wheel = vy_body + yaw_rate*VehicleParameters.a;
    vx_wheel = vx_body + yaw_rate*(VehicleParameters.trackwidth/2);
    Fz = NormalLoads(ax, ay, vx_wheel, "FR");
  elseif pos == WheelPosition.RL then
    vy_wheel = vy_body + yaw_rate*VehicleParameters.b;
    vx_wheel = vx_body - yaw_rate*(VehicleParameters.trackwidth/2);
    Fz = NormalLoads(ax, ay, vx_wheel, "RL");
  elseif pos == WheelPosition.RR then
    vy_wheel = vy_body + yaw_rate*VehicleParameters.b;
    vx_wheel = vx_body + yaw_rate*(VehicleParameters.trackwidth/2);
    Fz = NormalLoads(ax, ay, vx_wheel, "RR");
  end if;
  
  alpha = -atan(vy_wheel/vx_wheel) - delta;
  kappa = 0; // very simplified for steady-state
  
  (Fx, Fy) = MagicFormulaFxFy(kappa, alpha, Fz);
  
  // connecting to vehicle.mo
  delta_input.ang = delta;
  v_body.vx = vx_body;
  v_body.vy = vy_body;
  yaw_rate_input.ang_vel = yaw_rate;
  accel_input.ax = ax;
  accel_input.ay = ay;
  
  

end Wheel;
