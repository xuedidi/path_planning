% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function cost=segmentCost(n,newPos,map)
penalty=10000; % penlaty for infeasible segments in path
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
prev=n;
cost=0;
if sqrt(sum((n-newPos).^2))>1
    for r=1:0.5:sqrt(sum((n-newPos).^2))
        posCheck=n+r.*[sin(dir) cos(dir)];
        segmentDistance=distanceCost(prev,posCheck);
        if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
                feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
            % cost is path length outside obstacles with a heavy panelty propotional to the segment of path inside obstacles
            cost=cost+penalty*segmentDistance;
        else
            cost=cost+segmentDistance;
        end
        prev=posCheck;
    end
else
    segmentDistance=sqrt(sum((n-newPos).^2));
    if ~(feasiblePoint(ceil(newPos),map) && feasiblePoint(floor(newPos),map) && ... 
                feasiblePoint([ceil(newPos(1)) floor(newPos(2))],map) && feasiblePoint([floor(newPos(1)) ceil(newPos(2))],map))
            % cost is path length outside obstacles with a heavy panelty propotional to the segment of path inside obstacles
            cost=cost+penalty*segmentDistance;
        else
            cost=cost+segmentDistance;
    end
end