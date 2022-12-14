%quaternion rotation function
%rotate vector p1 by quaternion q
%resulting vector is p2
%if incoming vector p1 is 2 units long, it is assumed to be [0;x;y;0]
%if incoming vector p1 is 3 units long, it is assumed to be [0;x;y;z]

function p2 = quatRot(p1,q)
  sz = size(p1);
  if numel(p1)==2
    qp1 = [0,p1(1),p1(2),0];
  elseif numel(p1)==3
    qp1 = [0,p1(1),p1(2),p1(3)];
  elseif numel(p1)==4
    qp1 = [p1(1),p1(2),p1(3),p1(4)];
    warning('was not expecting vector to be rotated to have a scalar component')
  else
    error('odd number of elements in vector to be rotated')
  endif

  if numel(q)~=4
    error('Not enough elements in quaternion q')
  endif

  qinv = qconj(q);

  q2 = quatMult(quatMult(q,qp1),qinv);

  p1mag = norm(p1);

  if sz(1)==1
    p2 = [q2(2),q2(3),q2(4)];
  else
    p2 = [q2(2);q2(3);q2(4)];
  endif

  p2 = (p2/norm(p2))*p1mag;
endfunction

