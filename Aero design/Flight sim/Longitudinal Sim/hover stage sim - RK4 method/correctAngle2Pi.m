function B = correctAngle2Pi(A)
  while ~(A<(2*pi)&&A>=0)
    if A<0
      A = A+2*pi;
    elseif A>=(2*pi)
      A = A-2*pi;
    endif
  endwhile
  B = A;
endfunction

