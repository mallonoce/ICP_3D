function [xes,chis, Xfin]=pointAlignerLoop(x,Z, iterations)
  xes=zeros(6,iterations,2);
  chis=zeros(1,iterations);
  Xnew=fromVector(x);
  for i=1:iterations
    [Xnew,chiNew]=pointAlignerIteration(Xnew,Z);
    temp = toVector(Xnew);
    xes(:,i,1)=temp(:,1);
    xes(:,i,2)=temp(:,2);
    chis(1,i)=chiNew;
  end
  Xfin = Xnew;
endfunction