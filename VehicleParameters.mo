within;
package VehicleParameters
  import Modelica.Units.SI.*;
  constant Height h = 0.24; // cg height
  constant Length wheelbase = 1.53;
  constant Length trackwidth = 1.2;
  constant Length a = 0.80325; // cg to front axle length
  constant Length b = 0.72675; // cg to rear axle length
  constant Mass vehicleMass = 262; // TODO: does not include driver weight?
  constant Density rho_air = 1.293;
  constant Real Cd = 1.34458; // drag coef
  constant Real Cl = 3.3; // df/lift coef
  constant Real CLf = 1.65; // Cl for front
  constant Real CLr = 1.65;
  constant Area A = 1.1; // frontal area
  constant Real gamma = 0.5; // brake ratio
  constant Real roll_stiffness = 0.53;
  constant Power P_max = 80*1000; // max power in (W)  
  
  constant Real LLTD = 0.5; // lateral load transfer dist
  constant Real aeroDistro = 0.46;
  constant Real weightDistro = 0.475;
  
end VehicleParameters;
