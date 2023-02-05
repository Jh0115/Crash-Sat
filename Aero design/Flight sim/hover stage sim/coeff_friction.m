%vel - velocity magnitude
%chord - aircraft chord length

function [cf_lam,cf_turb] = coeff_friction(vel,chord,rho,mu)
  Re = vel*rho*chord/mu;
  Re_crit = 50000;
  c_transition = 0.7388/vel;

  if c_transition>chord
    cf_turb = 0;
  else
    cf_turb = 0.027/(Re)^(1/7);
  endif
  cf_lam = 0.664/sqrt(Re);

endfunction

