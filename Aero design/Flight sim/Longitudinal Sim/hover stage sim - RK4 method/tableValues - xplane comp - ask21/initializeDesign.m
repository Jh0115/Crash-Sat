%access the excel file lookup tables for external data. implement all singular constants in this function

function ac_struct = initializeDesign()
  pkg load io
  ac_struct.Sa = 17.95; %surface area in m^2
  ac_struct.c = 1.056; %mean chord
  ac_struct.AR = ac_struct.Sa/ac_struct.c/ac_struct.c; %wing aspect ratio
  ac_struct.oe = 0.7; %oswald efficiency factor THIS CAN BE APPROXIMATED THROUGH ASPECT RATIO BUT IM LEAVING IT THIS SIMPLE FOR NOW

  ac_struct.m = 400; %total system mass

  ac_struct.MOI_y = 600; %moment of inertia

  %excel files
  ac_struct.alpha = xlsread('lookupTableAlpha.xlsx');
  ac_struct.vel = xlsread('lookupTableVel.xlsx');
  ac_struct.dClda = xlsread('lookupTabledClda.xlsx');
  ac_struct.dCdda = xlsread('lookupTabledCdda.xlsx');
  ac_struct.dCmda = xlsread('lookupTabledCmda.xlsx');
  ac_struct.Cl0 = xlsread('lookupTableCl0.xlsx');
  ac_struct.Cd0 = xlsread('lookupTableCd0.xlsx');
  ac_struct.Cm0 = xlsread('lookupTableCm0.xlsx');

endfunction

