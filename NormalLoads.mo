within mbsSetupOptimizer;
function NormalLoads
  import Modelica.Constants.g_n;
  import Modelica.Units.SI.*;
  
  input Acceleration ax;
  input Acceleration ay;
  input Velocity u;
  input Integer wheelPos; // 'FL'=1, 'FR'=2, 'RL'=3, 'RR'=4
  
  output Force Fz; // TODO: I guess this should be Force not Real?
  
  protected
    Real FLf;
    Real FLr;
    
    constant Real Nmin = 10; // min Fz value permitted
    constant Real eps_smooth = 1.0; // smoothing tolerance
    
algorithm
  FLf := 0.5*VehicleParameters.CLfA*VehicleParameters.rho_air*(u^2);
  FLr := 0.5*VehicleParameters.CLrA*VehicleParameters.rho_air*(u^2);
  
  // TODO: this entire section can be easily simplified
  if wheelPos == 1 then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) - 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) + VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  elseif wheelPos == 2 then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.b/(VehicleParameters.a + VehicleParameters.b)) - 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) - VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLf;
    
  elseif wheelPos == 3 then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.a/(VehicleParameters.a + VehicleParameters.b)) + 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) + VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLr;

  elseif wheelPos == 4 then
    Fz := 0.5*VehicleParameters.vehicleMass*g_n*(VehicleParameters.a/(VehicleParameters.a + VehicleParameters.b)) + 0.5*VehicleParameters.vehicleMass*ax*(VehicleParameters.h/(VehicleParameters.a + VehicleParameters.b)) - VehicleParameters.vehicleMass*ay*(VehicleParameters.h/VehicleParameters.trackwidth)*VehicleParameters.roll_stiffness + 0.5*FLr;
    
  else
    Fz := 0; // should never get here
    
  end if;
    
  

end NormalLoads;
