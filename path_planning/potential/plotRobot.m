% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Artificial Potential Fields, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function feasible=plotRobot(position,direction,map,robotHalfDiagonalDistance)
% plot robot with specified configuration and check its feasibility
 corner1=position+robotHalfDiagonalDistance*[sin(direction-pi/4) cos(direction-pi/4)];
 corner2=position+robotHalfDiagonalDistance*[sin(direction+pi/4) cos(direction+pi/4)];
 corner3=position+robotHalfDiagonalDistance*[sin(direction-pi/4+pi) cos(direction-pi/4+pi)];
 corner4=position+robotHalfDiagonalDistance*[sin(direction+pi/4+pi) cos(direction+pi/4+pi)];
 line([corner1(2);corner2(2);corner3(2);corner4(2);corner1(2)],[corner1(1);corner2(1);corner3(1);corner4(1);corner1(1)],'color','blue','LineWidth',2);
 if ~feasiblePoint(int16(corner1),map) || ~feasiblePoint(int16(corner2),map) || ~feasiblePoint(int16(corner3),map) || ~feasiblePoint(int16(corner4),map)
     feasible=false;
 else
     feasible=true;
 end