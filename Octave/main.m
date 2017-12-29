clear
clc
# number of generated points
nPoints = 100;

# generate a random set of points
P1 = (rand(3,nPoints)-.5)*100;
# P2 = shuffleP(P1);
# create a guess transform around the z axis:
v = [10,20,30,pi/4,pi/4,pi/4 ];
A = fromVector(v);
# obtain the transformed points (by column)
Pt = transformPoints(inv(A),P1);

# assemble the measurement matrix
# first three rows x y and z of each "reference" point
# third, fourth and fifth row represent x, y and z of 
# each "current" point a column is a measurement;
Z=zeros(4,nPoints);
Z(1:3,:)=P1;
Z(4:6,:)=Pt;

x=[0,0,0,0,0,0]';
# now run the algorithm and get the chi2
[xes, chis, Xfin ] = pointAlignerLoop(x,Z,100)