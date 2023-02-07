function B = correctAnglePi(A)
  while ~(A<pi&&A>=-pi)
    if A>pi
      A = A-2*pi;
    elseif A<=-pi
      A = A+2*pi;
    endif
  endwhile
  B = A;
endfunction

