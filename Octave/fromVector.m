# compute the homogeneous transformation from 
# the pose vector v = [tx,ty,tz,psi,theta,phi]'
# using roll pitch yaw

function  A = fromVector(v)
   A(1:3,4)=v(1:3);
   cpsi = cos(v(4));
   spsi = sin(v(4));
   ct = cos(v(5));
   st = sin(v(5));
   cphi = cos(v(6));
   sphi = sin(v(6));
   
   A(1:3,1) = [cphi*ct;sphi*ct;-st];
   A(1:3,2) = [cphi*st*spsi-sphi*cpsi;sphi*st*spsi+cphi*cpsi;ct*spsi];
   A(1:3,3) = [cphi*st*cpsi+sphi*spsi;sphi*st*cpsi-cphi*spsi;ct*cpsi];
   A(4,:) = [0 0 0 1];

endfunction