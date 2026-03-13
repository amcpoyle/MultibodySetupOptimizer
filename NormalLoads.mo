function NormalLoads
  import Modelica.Constants.g_n;
  import Modelica.Units.SI.*;
  
  input Acceleration ax;
  input Acceleration ay;
  input Velocity u;
  input String wheelPos; // 'FL', 'FR', 'RL', 'RR'
  
  output Real Fz; // TODO: I guess this should be Force not Real?
  
  protected
    Real FLf;
    Real FLr;
    
    constant Real Nmin = 10; // min Fz value permitted
    constant Real eps_smooth = 1.0; // smoothing tolerance
    
algorithm
  FLf := 0.5*VehicleParameters.CLfA*VehicleParameters.rho_air*(u^2);
  FLr := 0.5*VehicleParameters.CLrA*VehicleParameters.rho_air*(u^2);
  
  // TODO: this entire section can be easily simplified
  if wheelPos == "FL" then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) - 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) + VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  elseif wheelPos == "FR" then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) - 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) - VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  elseif wheelPos == "RL" then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) + 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) + VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  elseif wheelPos == "RR" then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) + 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) - VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  else
    Fz := 0; // should never get here
    
  end if;
    
  

end NormalLoads;
