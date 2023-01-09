%%%%%%%%%%%%%%%%%%%%%%

% Created by : Payal Nandi
% Autonomous UnderWater Robotics 
% Instructor: Dr. Kaipa
% Guide: Ashish 

%%%%%%%%%%%%%%%%%%%%%%

clear all; clf; clc;
figure(1);
x0=100; y0=100; z0=100;

% Draw the sphere
r=[30;20;20;15;15];
circleCenter = [100,100,100;50,50,50;100,40,60;150,100,100;60,130,50];
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end

% Draw cyclinder

cylCenter = [300,400,0;450,50,50;150,40,60;150,100,100;60,130,50];
cylr=[10;15;10;10;10];
cylH =[10;20;25;15;30];
[x1,y1,z1]=cylinder;
for i = 1:length(cylCenter(:,1))
mesh(r(i)*x1+cylCenter(i,1),r(i)*y1+cylCenter(i,2),r(i)*z1*cylH(i)+cylCenter(i,3));hold on;
end
hold on;
axis equal

% Image scale to axis on the bottom
img = imread('LandMass2.jpg');            % Load a sample image
xImage = [-50 -50 ; 550 550];             % The x data for the image corners
yImage = [-50 550; -50 550];              % The y data for the image corners
zImage = [0 0; 0 0];                             % The z data for the image corners
surf(xImage,yImage,zImage,...             % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
axis equal


source=[10 10 10];   % Starting Node
goal=[320 400 200];  % Goal Node
stepsize = 10;
threshold = 10;
maxFailedAttempts = 1000000;
display = true;
searchSize = [800 800 800];    

% Ploting The Starting and End Goal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
text(source(1),source(2),source(3),'  Start');
text(goal(1),goal(2),goal(3),'  Goal');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Q-Learning with RRT Algorithm UAV planning path');

tic;  % tic-toc: Functions for Elapsed Time
QL = double([source -1]);
failedAttempts = 0;
pathFound = false;

%% Defiing loop
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand < 0.5
        sample = rand(1,3) .* searchSize;   % random sample
    else
        sample = goal; % sample taken as goal to bias tree generation to goal
    end
    %% selects the node in the QL tree that is closest to qrand
    
    [A, I] = min( distanceCost(QL(:,1:3),sample) ,[],1); % find the minimum value of each column
    closestNode = QL(I(1),1:3);
    %% moving from qnearest an incremental distance in the direction of qrand
    
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  
    newPoint = closestNode + stepsize * movingVec;
    
%     if ~checkPath3(closestNode, newPoint, circleCenter,r) % if extension of closest node in tree to the new point is feasible   
%         failedAttempts = failedAttempts + 1;
% %         continue;
    if ~checkPath4(closestNode, newPoint, cylCenter,cylr,cylH) % if extension of closest node in tree to the new point is feasible   
        failedAttempts = failedAttempts + 1;
       continue;
     
    end
%     end
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end % goal reached
    [A, I2] = min( distanceCost(QL(:,1:3),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,QL(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    QL = [QL; newPoint I(1)]; % add node
    failedAttempts = 0;
    if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1); end
    pause(0.05);
end
%%
if display && pathFound, plot3([closestNode(1);goal(1)],[closestNode(2);goal(2)],[closestNode(3);goal(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end



%% retrieve path from parent information
path = goal;
prev = I(1);
while prev > 0
    path = [QL(prev,1:3); path];
    prev = QL(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 

%% Display figure with the shortest path

figure(2)

for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end

% Draw cyclinder

for i = 1:length(cylCenter(:,1))
mesh(r(i)*x1+cylCenter(i,1),r(i)*y1+cylCenter(i,2),r(i)*z1*cylH(i)+cylCenter(i,3));hold on;
end
hold on;
axis equal

hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
text(source(1),source(2),source(3),'  Start');
text(goal(1),goal(2),goal(3),'  Goal');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Q-Learning with RRT Algorithm UAV planning path');

% Image scale to axis on the bottom
img = imread('LandMass2.jpg');            % Load a sample image
xImage = [-50 -50 ; 550 550];             % The x data for the image corners
yImage = [-50 550; -50 550];              % The y data for the image corners
zImage = [0 0; 0 0];                             % The z data for the image corners
surf(xImage,yImage,zImage,...             % Plot the surface
     'CData',img,...
     'FaceColor','texturemap');
axis equal


