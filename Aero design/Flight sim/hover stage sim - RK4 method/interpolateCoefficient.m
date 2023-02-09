function coeff_3 = interpolateCoefficient(ac_struct,coeff_mat,alpha_ind,vel_ind,alpha,vel)

  %step 1: load relevant values, nearest alpha, nearest vel
  LT_alpha_ref = ac_struct.alpha; %reference for alpha values of lookup tables
  LT_vel_ref = ac_struct.vel; %reference for vel values of lookup tables

  alpha_base = LT_alpha_ref(alpha_ind);
  vel_base = LT_vel_ref(vel_ind);

  %step 2: if actual alpha and vel are above, then load next highest also, if values are below load next lowest also
  %         and if there is no other value higher above or below, load the opposite by default

  if alpha>alpha_base
    if alpha_ind~=numel(LT_alpha_ref)
      alpha_ind_2 = alpha_ind+1;
    else
      alpha_ind_2 = alpha_ind-1;
    endif
  else
    if alpha_ind~=1
      alpha_ind_2 = alpha_ind-1;
    else
      alpha_ind_2 = alpha_ind+1;
    endif
  endif

  if vel>vel_base
    if vel_ind~=numel(LT_vel_ref)
      vel_ind_2 = vel_ind+1;
    else
      vel_ind_2 = vel_ind-1;
    endif
  else
    if vel_ind~=1
      vel_ind_2 = vel_ind-1;
    else
      vel_ind_2 = vel_ind+1;
    endif
  endif

  alpha_2 = LT_alpha_ref(alpha_ind_2);
  vel_2 = LT_vel_ref(vel_ind_2);

  %step 3: interpolate along the alpha_base and alpha_2 axes to narrow down the interpolation problem
  %get C_int_1, and C_int_2
  C_int_1 = interExterpolate(alpha_base,coeff_mat(alpha_ind,vel_ind),alpha_2,coeff_mat(alpha_ind_2,vel_ind_2),alpha);
  C_int_2 = interExterpolate(alpha_base,coeff_mat(alpha_ind,vel_ind_2),alpha_2,coeff_mat(alpha_ind_2,vel_ind_2),alpha);

  coeff_3 = interExterpolate(vel_base,C_int_1,vel_2,C_int_2,vel);

endfunction

