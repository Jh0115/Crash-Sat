%function for finding the value and index of vector A which closest matches a provided value, B

function [ind,val] = findClosest1D(A,B)
  x = abs(A-B);
  x_mins = find(x==min(x));
  ind = x_mins(1);
  val = A(ind);
endfunction





