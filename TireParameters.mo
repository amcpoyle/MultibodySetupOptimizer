within mbsSetupOptimizer;
package TireParameters
  constant Real ref_load = 1500;
  constant Real pCx1 = 1.532;
  constant Real pDx1 = 2.0217;
  constant Real pDx2 = -1.3356*(10^(-12));
  constant Real pEx1 = -0.53967;
  constant Real pKx1 = 31.5328;
  constant Real pKx3 = -0.83511;
  constant Real lambda_mux = 1;
  
  constant Real pCy1 = 1.5;
  constant Real pDy1 = 2.3298;
  constant Real pDy2 = -0.5;
  constant Real pEy1 = -0.052474;
  constant Real pKy1 = -42.8074;
  constant Real pKy2 = 1.7679;
  constant Real lambda_muy = 1;
  
  constant Real mu = 1; // road friction coef
end TireParameters;
