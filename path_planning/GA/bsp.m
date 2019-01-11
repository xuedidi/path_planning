% © Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Genetic Algorithms, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function pathSmooth=bsp(path) % function takes a set of points and returns a smooth spline curve from those points
x=linspace(0,1,length(path));
path=double(path);
pp=spline(x,path');
pathSmooth = double(int32(ppval(pp, linspace(0,1,10000))))';
