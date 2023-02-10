%vel - velocity magnitude
%chord - aircraft chord length

function [cf_lam,cf_turb,cf_times_Sa] = coeff_friction(vel,chord,rho,mu,Sa)
  Re = vel*rho*chord/mu;
  Re_crit = 500000;
  c_transition = Re_crit*mu/(vel*rho);

  if c_transition>chord
    cf_turb = 0;
  else
    cf_turb = 0.027./(Re).^(1/7);
  endif
  cf_lam = 0.664./sqrt(Re);
  
  perc_lam = c_transition/chord;
  perc_turb = 1-perc_lam;
  
  cf_times_Sa = cf_lam*Sa*perc_lam+cf_turb*Sa*perc_turb;

endfunction

