% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

global map source goal splineSmoothing

map=im2bw(imread('map1.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
noOfPointsInSolution=3; % no. of points that represent a candidate path, excluding the source and goal. each point marks a robot turn.
NoOfGenerations=10;
PopulationSize=50;
splineSmoothing=true; % use spline based smoothing. the code has a dependence on the resoultion, and may be set to false if large changes in resolution are made.

%%%%% parameters end here %%%%%

tic;
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if noOfPointsInSolution<=0, error('noOfPointsInSolution should be greater than 1'); end

% currently the lower bounds and upper bounds are taken as 0 and 1 respectively, 
% these would be re-scaled in phenotype generation to ensure that they lie inside map. 
options=gaoptimset('PlotFcns', {@gaplotbestf, @gaplotdistance, @gaplotrange},'Generations',NoOfGenerations,'PopulationSize',PopulationSize);
[solution cost] = ga(@PathCostGA, noOfPointsInSolution*2,[],[],[],[],zeros(noOfPointsInSolution*2,1),ones(noOfPointsInSolution*2,1),[],options);
disp('click/press any key');
waitforbuttonpress; 
if PathCostGA(solution)>size(map,1)*size(map,2) % indicating an infeasible path due to large cost due to penalties
    error('no path found');
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,cost); 
path=[source; [solution(1:2:end)'*size(map,1) solution(2:2:end)'*size(map,2)]; goal]; % souce and goal is fixed. other points are from the GA individual representation
if splineSmoothing
    path=bsp(path); % a point based specification of path is smoothed by using splines
end
clf;
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));
