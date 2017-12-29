#apply the transformation matrix X to the 3D points P
function Pt=transformPoints(X,P)
  Pfull = zeros(4,size(P)(2));
  Pfull(1:3,:)=P;
  Pfull(4,:)=1;
  Pfull=X*Pfull;
  Pt=Pfull(1:3,:);
endfunction