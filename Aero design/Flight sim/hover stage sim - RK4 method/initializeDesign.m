%access the excel file lookup tables for external data. implement all singular constants in this function

function ac_struct = initializeDesign()

  rho = 1.225;
  mu = 0.0000181;

  ac_struct.Sa = 17.95; %surface area in m^2
  ac_struct.c = 1.056; %mean chord
  ac_struct.AR = ac_struct.Sa/ac_struct.c/ac_struct.c; %wing aspect ratio
  ac_struct.oe = 0.7; %oswald efficiency factor THIS CAN BE APPROXIMATED THROUGH ASPECT RATIO BUT IM LEAVING IT THIS SIMPLE FOR NOW

  ac_struct.m = 400; %total system mass

  ac_struct.MOI_y = 400; %moment of inertia

  %excel files
  pkg load io
  ac_struct.alpha = xlsread('lookupTableAlpha.xlsx');
  ac_struct.vel = xlsread('lookupTableVel.xlsx');

  %alpha and vel for calculations
  [vel,aoa] = meshgrid(ac_struct.vel,ac_struct.alpha);

  %lookup tables
  ac_struct.Cl = xlsread('lookupTableCl.xlsx');
  %ac_struct.Cm = xlsread('lookupTableCm.xlsx');
  %ac_struct.dClda = xlsread('lookupTabledClda.xlsx');
  ac_struct.dCmda = xlsread('lookupTabledCmda.xlsx');
  %ac_struct.Cl0 = xlsread('lookupTableCl0.xlsx');
  ac_struct.Cm0 = xlsread('lookupTableCm0.xlsx');

  [cf_lam,cf_turb] = coeff_friction(vel,ac_struct.c,rho,mu);
  ac_struct.Cd = cf_lam+cf_turb+(ac_struct.Cl.*ac_struct.Cl./(pi*ac_struct.AR*ac_struct.oe));

  %ac_struct.dCdda = xlsread('lookupTabledCdda.xlsx');
  %ac_struct.Cd0 = xlsread('lookupTableCd0.xlsx');

endfunction
