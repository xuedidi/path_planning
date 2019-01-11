% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Artificial Potential Fields, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

map=int16(im2bw(imread('map1.bmp'))); % input map read from a bmp file. for new maps write the file name here
source=[50 50]; % source position in Y, X format
goal=[450 450]; % goal position in Y, X format
robotDirection=pi/8; % initial heading direction
robotSize=[10 10]; %length and breadth
robotSpeed=10; % arbitrary units 
maxRobotSpeed=10; % arbitrary units 
S=10; % safety distance
distanceThreshold=30; % a threshold distace. points within this threshold can be taken as same. 
maxAcceleration=10; % maximum speed change per unit time
maxTurn=10*pi/180; % potential outputs to turn are restriect to -60 and 60 degrees.
k=3; % degree of calculating potential
attractivePotentialScaling=300000; % scaling factor for attractive potential
repulsivePotentialScaling=300000; % scaling factor for repulsive potential
minAttractivePotential=0.5; % minimum attractive potential at any point

%%%%% parameters end here %%%%%

currentPosition=source; % position of the centre of the robot
currentDirection=robotDirection; % direction of orientation of the robot
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % used for distance calculations 
pathFound=false; % has goal been reached
pathCost=0;
t=1;
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
pathLength=0; 
if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
     error('source lies on an obstacle or outside map'); 
end
M(t)=getframe;
t=t+1;

if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end

tic;
while ~pathFound
    
    % calculate distance from obstacle at front
    i=robotSize(1)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection) cos(currentDirection)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
    
    % calculate distance from obstacle at left
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    % calculate distance from obstacle at right
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    % calculate distance from obstacle at front-left diagonal
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontLeftDiagonal=i-robotHalfDiagonalDistance;
    
    % calculate distance from obstacle at front-right diagonal
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontRightDiagonal=i-robotHalfDiagonalDistance;
    
    % calculate angle from goal
     angleGoal=atan2(goal(1)-currentPosition(1),goal(2)-currentPosition(2));
    
     % calculate diatnce from goal
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     if distanceGoal<distanceThreshold, pathFound=true; end
     
     % compute potentials
     repulsivePotential=(1.0/distanceFront)^k*[sin(currentDirection) cos(currentDirection)] + ...
     (1.0/distanceLeft)^k*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)] + ...
     (1.0/distanceRight)^k*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)] + ...
     (1.0/distanceFrontLeftDiagonal)^k*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)] + ...
     (1.0/distanceFrontRightDiagonal)^k*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)];
     
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential])*[sin(angleGoal) cos(angleGoal)];
     totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     % perform steer
     preferredSteer=atan2(robotSpeed*sin(currentDirection)+totalPotential(1),robotSpeed*cos(currentDirection)+totalPotential(2))-currentDirection;
     while preferredSteer>pi, preferredSteer=preferredSteer-2*pi; end % check to get the angle between -pi and pi
     while preferredSteer<-pi, preferredSteer=preferredSteer+2*pi; end % check to get the angle between -pi and pi
     preferredSteer=min([maxTurn preferredSteer]);
     preferredSteer=max([-maxTurn preferredSteer]);
     currentDirection=currentDirection+preferredSteer;
     
     % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
     preferredSpeed=sqrt(sum((robotSpeed*[sin(currentDirection) cos(currentDirection)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     if robotSpeed==0, error('robot had to stop to avoid collission'); end
     
     % calculating new position based on steer and speed
     newPosition=currentPosition+robotSpeed*[sin(currentDirection) cos(currentDirection)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     if ~feasiblePoint(int16(currentPosition),map), error('collission recorded'); end
     
     % plotting robot
     if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
        error('collission recorded');
     end
     M(t)=getframe;t=t+1;
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathCost); 