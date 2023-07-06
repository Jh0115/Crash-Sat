%interpolate or exterpolate linearly to x3 using coords x1,x2,y1,y2

function y3 = interExterpolate(x1,y1,x2,y2,x3)

  y3 = ((y2-y1)/(x2-x1))*(x3-x2)+y2;

endfunction


