clc
% R=10;
% rA =[300,400,0];
% rB =[300,400,300];
% rP = [150 150 550];
% checkPathcyl
e = rB-rA;
m = cross(rA,rB);
d = (m+cross(e,rP))/(norm(e))

rQ = rP + (cross(e,(m+cross(e,rP))))/(norm(e))^2

wA =norm( cross(rQ,rB))/norm(m)
wB =norm( cross(rQ,rA))/norm(m)

if(d<R & wA>=0 & wA <=1 & wB>=0 & wB<=1)
    disp('inside')
else
    disp('outside')
end