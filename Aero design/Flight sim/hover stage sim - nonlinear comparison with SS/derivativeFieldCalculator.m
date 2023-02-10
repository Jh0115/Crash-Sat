function dAdB = derivativeFieldCalculator(A,B)
  %given a field of scalar values A, with axes B and C, calculator a scalar field of double partial derivatives
  dB = diff(B);

  %for first and last item use a forward difference and backwards difference method
  dAdB_first = (A(2,:)-A(1,:))/dB(1);
  dAdB_last = (A(end,:)-A((end-1),:))/dB(end);

  %for all other points use a centered difference method
  dB_center = mean([dB(1:end-1,:),dB(2:end,:)],2);
  dAdB_center = ((A((3:end),:))-(A((1:end-2),:)))./(2*dB_center);

  %combine into one field
  dAdB = [dAdB_first;dAdB_center;dAdB_last];

endfunction

