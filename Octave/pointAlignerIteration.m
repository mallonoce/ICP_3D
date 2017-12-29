function [XNew, chi]=pointAlignerIteration(X,Z)
  H=zeros(6,6);
  b=zeros(6,1);
  chi=0;
  #loop through the measurements and update the
  #accumulators
  for i=1:size(Z,2),
    e=computeError(i,X,Z);
    J=computeJacobian(i,X,Z);
    H+=J'*J;
    b+=J'*e;
    chi+=e'*e;
  end
  dx=-H\b;
  dX = fromVector(dx);
  XNew = dX*X;
endfunction
