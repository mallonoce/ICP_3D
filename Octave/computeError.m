# e(X boxPlus dx) = fromVector(dx)*X*pi-zi
# see notes pag.19
function e=computeError(i,X,Z)
  pi=ones(4,1);
  pj=ones(4,1);
  pi(1:3)=Z(1:3,i);
  pj(1:3)=Z(4:6,i);
  efull = pi-X*pj;
  e=efull(1:3);
endfunction