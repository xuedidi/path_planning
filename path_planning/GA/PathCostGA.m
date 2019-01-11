% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function cost=PathCostGA(X) %GA cost function
global map source goal splineSmoothing

path=[source; [X(1:2:end)'*size(map,1) X(2:2:end)'*size(map,2)]; goal]; % souce and goal is fixed. other points are from the GA individual representation

if splineSmoothing
    path=bsp(path); % a point based specification of path is smoothed by using splines
end

cost=0;
for i=2:length(path)
    cost=cost+segmentCost(path(i-1,:),path(i,:),map);
end