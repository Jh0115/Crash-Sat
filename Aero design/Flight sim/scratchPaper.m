clearho all
close all
clc

w-(((rhoSa)/(2m))*(x*((dCC_L0daoa*aoa+C_L0)cos(th-aoa))-((dCC_D0daoa*aoa+C_D0)sin(th-aoa))-y*((dCC_L0daoa*aoa+C_L0)sin(th-aoa))-((dCC_D0daoa*aoa+C_D0)cos(th-aoa)))/(x^2))

(Sa*rho*(((vx*dCLdaoa-dCDdaoa)*aoa-vy*dCLdaoa+C_L0*vx-dCDdaoa-C_D0)*sin(aoa-th)+((-vy*dCLdaoa-dCDdaoa)*aoa-vx*dCLdaoa-C_L0*y+dCDdaoa-C_D0)*cos(aoa-th)))/(2*m*x^2)
