function qinv = qconj(q)
  sz = size(q);
  if sz(1)==4
    qinv = [q(1);
            -q(2);
            -q(3);
            -q(4)];
  elseif sz(2)==4
    qinv = [q(1),-q(2),-q(3),-q(4)];
  else
    error('Input quaternion is not 4 elements long')
  endif
endfunction


