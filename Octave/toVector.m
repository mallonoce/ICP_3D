# compute the pose vector v = [tx,ty,tz,psi,theta,phi]'
# from the homogeneous transformation 

function  v = toVector(A)

   v(1:3,1) = A(1:3,4);
   v(1:3,2) = v(1:3,1);
   v(5,1) = atan2(-A(3,1),sqrt(A(3,2)^2+A(3,3)^2));
   v(5,2) = atan2(-A(3,1),-sqrt(A(3,2)^2+A(3,3)^2));
   
   if(abs(v(5) = pi/2))
      v(4,1) = atan2(A(3,2)/cos(v(5,1)),A(3,3)/cos(v(5,1)));
      v(6,1) = atan2(A(2,1)/cos(v(5,1)),A(1,1)/cos(v(5,1)));
      v(4,2) = atan2(A(3,2)/cos(v(5,2)),A(3,3)/cos(v(5,2)));
      v(6,2) = atan2(A(2,1)/cos(v(5,2)),A(1,1)/cos(v(5,2)));
   else
      'ERROR theta angle = pi/2'
   endif   

endfunction