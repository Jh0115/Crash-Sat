%function for calculating AOA given orientation and velocity vector using vector definition
function aoa = calculateAOA(O,V)
  magO = sqrt(O(1)^2+O(2)^2);
  magV = sqrt(V(1)^2+V(2)^2);

  cosAOA = dot(O,V)/(magO*magV);

  aoa_unsigned = acos(cosAOA); %this is the angle magnitude but the function doesnt know if it is positive or negative

  if numel(O)<3
    O(3) = 0;
  endif

  if numel(V)<3
    V(3) = 0;
  endif

  signature = cross(V,O);

  if signature(3)>=0
    aoa = abs(aoa_unsigned);
  elseif signature(3)<0
    aoa = -abs(aoa_unsigned);
  endif

endfunction

