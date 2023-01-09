%% feasiblePoint3.m
function feasible=feasiblePoint3(point,circleCenter,r)
feasible=true;

% check if collission-free spot and inside maps
for row = 1:length(circleCenter(:,1))
    if distanceCost(circleCenter(row,:) ,point) <= r(row)
        feasible = false;break;
    end
end


