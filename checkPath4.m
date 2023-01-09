function feasible=checkPath4(n,newPos,cylCenter,cylr,cylH)
feasible=true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2),newPos(3)-n(3)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); %Unitization or Normalizing
for R=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n + R .* movingVec;
    if ~(feasiblePoint4(ceil(posCheck),cylCenter,cylr,cylH) && feasiblePoint4(floor(posCheck),cylCenter,cylr,cylH))
        feasible=false;break;
    end
end
if ~feasiblePoint4(newPos,cylCenter,cylr,cylH), feasible=false; end
end