function J=computeJacobian(i,X,Z)
  J=zeros(3,6);
  J(1:3,1:3)=-eye(3);

 
 # derivative of the rotation matrix around the x axis
  Rprimex=[0, 0, 0; 0, 0, -1; 0, 1, 0]; 
 # derivative of the rotation matrix around the y axis
  Rprimey=[0, 0, 1; 0, 0, 0; -1, 0, 0];
 # derivative of the rotation matrix around the z axis
  Rprimez=[0, -1, 0; 1, 0, 0; 0, 0, 0];

  pj = [Z(4:6,i);1];
  pj = X*pj; # transform pj according to X
  J(1:3,4)=-Rprimex * pj(1:3);
  J(1:3,5)=-Rprimey * pj(1:3);
  J(1:3,6)=-Rprimez * pj(1:3);
  
endfunction;