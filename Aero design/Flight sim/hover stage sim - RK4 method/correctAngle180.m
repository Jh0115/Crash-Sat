function B = correctAngle180(A)
  while ~(A<180&&A>=-180)
    if A>180
      A = A-360;
    elseif A<=-180
      A = A+360;
    endif
  endwhile
  B = A;
endfunction

