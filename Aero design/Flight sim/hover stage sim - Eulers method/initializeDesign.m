%access the excel file lookup tables for external data. implement all singular constants in this function

function ac_struct = initializeDesign()
  pkg load io
  ac_struct.Sa = 0.25; %surface area in m^2
  ac_struct.c = 0.15; %mean chord

  ac_struct.m = 10; %total system mass

  ac_struct.MOI_y = 0.8667; %moment of inertia

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

