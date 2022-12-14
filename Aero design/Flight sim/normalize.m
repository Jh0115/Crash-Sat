function u = normalize(v)
  mag = norm(v);
  u = [v(1)/mag;v(2)/mag;v(3)/mag];
endfunction

