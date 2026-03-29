within mbsSetupOptimizer;
model Wheel
  import Modelica.Units.SI.*;
  
  // type WheelPosition = enumeration(FL, FR, RL, RR);
  parameter Integer pos; // 1 = FL, 2 = FR, 3 = RL, 4 = RR
  Angle delta(start=0); // steer angle of the wheel
  // steer angle of the wheel is determined by the angle of the steering wheel
  Angle delta_steering_wheel(start=0);
  AngularVelocity yaw_rate(start=0); // yaw rate
  AngularVelocity omega(start=0); // angular velocity of the wheel
  
  Velocity vx_wheel(start=0); // lon velocity
  Velocity vy_wheel(start=0); // lat velocity
  Velocity vx_body(start=0);
  Velocity vy_body(start=0);
  Acceleration ax(start=0);
  Acceleration ay(start=0);
  
  Force Fx(start=0);
  Force Fy(start=0);
  Force Fz(start=0);
  
  Real sigma_x(start=0); // long slip
  Real sigma_y(start=0); // lat slip

  
  // input connectors
  // we need steer angle of the wheel, body u and v, yaw rate
  // mbsSetupOptimizer.connectors.AngleInput delta_steering_wheel_input;
  // mbsSetupOptimizer.connectors.VelocityInput v_body;
  // mbsSetupOptimizer.connectors.AngularVelocityInput yaw_rate_input;
  // mbsSetupOptimizer.connectors.AccelerationInput accel_input;
  
equation
  delta = delta_steering_wheel/VehicleParameters.steer_ratio;
  omega = vx_wheel/VehicleParameters.rolling_radius;
  if pos == 1 then
    // FL
    vy_wheel = vy_body + yaw_rate*VehicleParameters.a;
    vx_wheel = vx_body - yaw_rate*(VehicleParameters.trackwidth/2);
    
    sigma_x = (vx_wheel - omega*VehicleParameters.rolling_radius)/(omega*VehicleParameters.rolling_radius); // lon slip
    sigma_y = (vy_wheel*cos(delta) - vx_wheel*sin(delta))/(vx_wheel*cos(delta) + vy_wheel*sin(delta));
    
    Fz = NormalLoads(ax, ay, vx_wheel, pos);
    
  elseif pos == 2 then
    // FR
    vy_wheel = vy_body + yaw_rate*VehicleParameters.a;
    vx_wheel = vx_body + yaw_rate*(VehicleParameters.trackwidth/2);
    
    sigma_x = (vx_wheel - omega*VehicleParameters.rolling_radius)/(omega*VehicleParameters.rolling_radius); // lon slip
    sigma_y = (vy_wheel*cos(delta) - vx_wheel*sin(delta))/(vx_wheel*cos(delta) + vy_wheel*sin(delta));
    
    Fz = NormalLoads(ax, ay, vx_wheel, pos);
    
  elseif pos == 3 then
    // RL
    vy_wheel = vy_body - yaw_rate*VehicleParameters.b;
    vx_wheel = vx_body - yaw_rate*(VehicleParameters.trackwidth/2);
    
    sigma_x = (vx_wheel - omega*VehicleParameters.rolling_radius)/(omega*VehicleParameters.rolling_radius); // lon slip
    sigma_y = vy_wheel/(omega*VehicleParameters.rolling_radius); // lat slip

    Fz = NormalLoads(ax, ay, vx_wheel, pos);

  elseif pos == 4 then
    // RR
    vy_wheel = vy_body - yaw_rate*VehicleParameters.b;
    vx_wheel = vx_body + yaw_rate*(VehicleParameters.trackwidth/2);

    sigma_x = (vx_wheel - omega*VehicleParameters.rolling_radius)/(omega*VehicleParameters.rolling_radius); // lon slip
    sigma_y = vy_wheel/(omega*VehicleParameters.rolling_radius); // lat slip
    
    Fz = NormalLoads(ax, ay, vx_wheel, pos);
  end if;

  // longitudinal slip calculations
  
  // lateral slip calculations

  
  (Fx, Fy) = MagicFormulaFxFy(sigma_x, sigma_y, Fz);
  
  // connecting to vehicle.mo
  // delta_steering_wheel_input.ang = delta_steering_wheel;
  // v_body.vx = vx_body;
  // v_body.vy = vy_body;
  // yaw_rate_input.ang_vel = yaw_rate;
  // accel_input.ax = ax;
  // accel_input.ay = ay;
  
  

end Wheel;
