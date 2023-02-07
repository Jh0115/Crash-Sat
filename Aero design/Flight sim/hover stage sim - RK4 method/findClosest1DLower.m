%function which, given an array of number A, and a value B, will find the 2 closest values and return the numerically lower one

function [ind,val] = findClosest1DLower(A,B)
  x = abs(A-B);
  x_mins = find(x==min(x));
  ind = x_mins(1);
  val = A(ind);
endfunction




