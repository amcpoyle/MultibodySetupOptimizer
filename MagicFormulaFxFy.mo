within mbsSetupOptimizer;
function MagicFormulaFxFy
  import Modelica.Units.SI.*;
  input Real kappa;
  input Angle alpha;
  input Force Fz;
  output Force Fx;
  output Force Fy;
  
  protected
    Real dfz;
    Real Kx;
    Real Ex;
    Real Dx;
    Real Cx;
    Real Bx;
    
    Real Ky;
    Real Ey;
    Real Dy;
    Real Cy;
    Real By;
    
    Real sig_x;
    Real sig_y;
    Real sig;
  
    constant Real eps = 1e-4;
    
  
algorithm
  dfz := (Fz - TireParameters.ref_load)/TireParameters.ref_load;
  
  Kx := Fz*TireParameters.pKx1*(1 + TireParameters.pKx3*dfz);
  Ex := TireParameters.pEx1;
  Dx := (TireParameters.pDx1 + TireParameters.pDx2*dfz)*TireParameters.lambda_mux;
  Cx := TireParameters.pCx1;
  Bx := Kx/(Cx*Dx + 1e-6);
  
  Ky := TireParameters.ref_load*TireParameters.pKy1*sin(2*atan(Fz/TireParameters.pKy2*TireParameters.ref_load));
  Ey := TireParameters.pEy1;
  Dy := (TireParameters.pDy1 + TireParameters.pDy2*dfz)*TireParameters.lambda_muy;
  Cy := TireParameters.pCy1;
  By := Ky/(Cy*Dy + 1e-6);
  
  sig_x := kappa/(1 + kappa);
  sig_y := alpha/(1 + kappa);
  sig := sqrt((sig_x^2) + (sig_y^2) + eps);
  
  Fx := Fz*(sig_x/sig)*Dx*sin(Cx*atan(Bx*sig - Ex*(Bx*sig - atan(Bx*sig))));
  Fy := Fz*(sig_y/sig)*Dy*sin(Cy*atan(By*sig - Ey*(By*sig - atan(By*sig))));

end MagicFormulaFxFy;
