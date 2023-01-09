
%%

clear all; clf; clc;
 npoints = [ 11.4368   12.9384   19.4499;...
              4.0427   15.9098   28.6358;...
             18.2557   18.6446   37.2828;...
             23.4963   22.1026   45.0661;...
             31.3995   30.4214   61.4470;...
             35.3512   34.5808   69.6375;...
             39.3028   38.7403   77.8280;...
             43.5766   43.6069   85.4471;...
             150       100       80;...
             347.4479  116.2620   53.2565];
r=[130];
cylCenter = [350,40,0];
cylTCenter = [350,40,85;];
cylH =[.5];

[x1,y1,z1]=cylinder;
for i = 1:length(cylCenter(:,1))
mesh(r(i)*x1+cylCenter(i,1),r(i)*y1+cylCenter(i,2),r(i)*z1*cylH(i)+cylCenter(i,3));hold on;
h = r(i)*z1*cylH(i)+cylCenter(i,3);
end
hold on;

xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
axis([-10 500 -10 500  -10 500])
axis equal

rA = [350,40,0];    % point A
rB = [350,40,85];    % point B
rP = npoints(10,:); % Any random point
plot3(rP(1),rP(2),rP(3),'r*')

d = rA - rB;  % direction vector or the axis of cyln
A = rA;
AP = rA -rP;
APcrossdnormed = norm(cross(AP,d));
dnormed = norm(d);
perdis = APcrossdnormed/dnormed


if (perdis< r)
    
    if (rP(3)>h(2,1))
        disp('out')
    else
        disp('insi')
    end
elseif (perdis > r) & (rP(3)<h(2,1))
        disp('outside')
elseif (perdis > r) & (rP(3)>h(2,1))
        disp('outside')
else
    disp('outside and below')
end

