function feasible=feasiblePoint4(point,cylCenter,cylr, cylH)
feasible=true;

% check if collission-free spot and inside maps

for i = 1:length(cylCenter(:,1))
     if distanceCost(cylCenter(i,:) , point) <= cylr(i) 
        if point(3) == cylH(i) && point(3) <= cylH(i)
         feasible  = false; break;
        end
    end
end
 end


