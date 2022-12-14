function q = vec2quat(p1,p2)
  %calculate the quaternion needed to rotate vector p1 to vector p2, such that q*p1*qinv = p2

  sz1 = size(p1);
  sz2 = size(p2);

  if numel(p1)==2 && sz1(1)==1
    p1 = [p1(1),p1(2),0];
  elseif numel(p1)==2 && sz1(2)==1
    p1 = [p1(1);p1(2);0];
  endif

  if numel(p2)==2 && sz2(1)==1
    p2 = [p2(1),p2(2),0];
  elseif numel(p2)==2 && sz2(2)==1
    p2 = [p2(1);p2(2);0];
  endif

  if sz1(1)~=sz2(1)
    p2 = p2';
  endif

  theta = acos(dot(p1,p2)/(norm(p1)*norm(p2)));
  AOR = cross(p1,p2);
  AOR = AOR/norm(AOR);

  w = cos(theta/2);
  x = sin(theta/2)*AOR(1);
  y = sin(theta/2)*AOR(2);
  z = sin(theta/2)*AOR(3);

  if sz1(1)==1
    q = [w,x,y,z];
  else
    q = [w;x;y;z];
  endif

  q = q/norm(q);

endfunction

