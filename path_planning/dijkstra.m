clc;
clear;
close all;
 uicontrol('Style','pushbutton','String','again', 'FontSize',12, ...
       'Position', [1 1 60 40], 'Callback','dijkstra');
%% set up color map for display 
cmap = [1 1 1; ...%  1 - white - 空地
        0 0 0; ...% 2 - black - 障碍 
        1 0 0; ...% 3 - red - 已搜索过的地方
        0 0 1; ...% 4 - blue - 下次搜索备选中心 
        0 1 0; ...% 5 - green - 起始点
        1 1 0;...% 6 - yellow -  到目标点的路径 
       1 0 1];% 7 - -  目标点 
colormap(cmap); 
map1 = zeros(10); 
wallpercent=0.4;
% % 设置障障碍 
map1(1:5, 7) = 2;
map1(8,1:3) = 2; 
map1(2:5,3:5)=2;
%map1(ceil(10^2.*rand(floor(10*10*wallpercent),1))) =2;
%  map(ceil(10.*rand),ceil(10.*rand)) = 5; % 起始点
%map(ceil(10.*rand),ceil(10.*rand)) = 6; % 目标点
% %% 建立地图
nrows = 10; 
ncols = 10;  
start_node = sub2ind(size(map1), 10,1); 
dest_node = sub2ind(size(map1),1,4); 
map1(dest_node) = 7;
% % 距离数组初始化
distanceFromStart = Inf(nrows,ncols);  
distanceFromStart(start_node) = 0; 
distanceFromgoal = Inf(nrows,ncols);  
distanceFromgoal(dest_node) = 0; 
% % 对于每个格单元，这个数组保存其父节点的索引。 
parent = zeros(nrows,ncols); 
% % 主循环
writerObj = VideoWriter('Dijkstra.avi');
open(writerObj);
tic
 while true 
  % 画出现状图
  map1(start_node) = 5;
  map1(dest_node) = 7;
  image(1.5, 1.5, map1); 
  grid on; 
  axis image; 
  drawnow; 
   % 找到距离起始点最近的节点
  [min_dist, current] = min(distanceFromStart(:)); %返回当前距离数组(距离起点）的最小值和最小值的位置索引。
  %[min_dist1, current1] = min(distanceFromgoal(:)); %返回当前距离数组（距离目标点）的最小值和最小值的位置索引。
   if ((current == dest_node) || isinf(min_dist)) %搜索到目标点或者全部搜索完，结束循环。
         break; 
   end; 
% if((current==current1)|| isinf(min_dist)|| isinf(min_dist1))
%       break;
% end      
 map1(current) = 3; %将当前颜色标为红色。
 %map1(current1)=3;
distanceFromStart(current) = Inf;  %当前区域在距离数组中设置为无穷，表示已搜索。
%distanceFromgoal(current1) = Inf;  %当前区域在距离数组中设置为无穷，表示已搜索。
 [i, j] = ind2sub(size(distanceFromStart), current); %返回当前位置的坐标
 %[i1, j1] = ind2sub(size(distanceFromgoal), current1); %返回当前位置的坐标
 neighbor = [ 
            i-1,j;... 
            i+1,j;... 
            i,j+1;... 
             i,j-1]; %确定当前位置的上下左右区域。
% neighbor2 = [ 
%             i1-1,j1;... 
%             i1+1,j1;... 
%             i1,j1+1;... 
%              i1,j1-1]; %确定当前位置的上下左右区域。        
      neighbor1 = [ 
              i-1,j-1;...
              i+1,j+1;...
              i-1,j+1;...
              i+1,j-1]; %确定当前位置的对角区域。      
 outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) +...
                    (neighbor(:,2)<1) + (neighbor(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。   
  outRangetest1 = (neighbor1(:,1)<1) + (neighbor1(:,1)>nrows) +...
                     (neighbor1(:,2)<1) + (neighbor1(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。        
% outRangetest2 = (neighbor2(:,1)<1) + (neighbor2(:,1)>nrows) +...
%                    (neighbor2(:,2)<1) + (neighbor2(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。
 locate = find(outRangetest>0); %返回超限点的行数。
   locate1 = find(outRangetest1>0); %返回超限点的行数。
 %locate2 = find(outRangetest2>0); %返回超限点的行数。
 neighbor(locate,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
 neighborIndex = sub2ind(size(map1),neighbor(:,1),neighbor(:,2)); %返回下次搜索区域的索引号。
   neighbor1(locate1,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
    neighborIndex1 = sub2ind(size(map1),neighbor1(:,1),neighbor1(:,2)); %返回下次搜索区域的索引号。
%     neighbor2(locate2,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
%     neighborIndex2 = sub2ind(size(map1),neighbor2(:,1),neighbor2(:,2)); %返回下次搜索区域的索引号。

 for i=1:length(neighborIndex) 
 if (map1(neighborIndex(i))~=2) && (map1(neighborIndex(i))~=3 && map1(neighborIndex(i))~= 5) 
     map1(neighborIndex(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
     if((neighborIndex(i)+1==current)||(neighborIndex(i)-1==current))
        if distanceFromStart(neighborIndex(i))> min_dist + 2  
          distanceFromStart(neighborIndex(i)) = min_dist+2; 
             parent(neighborIndex(i)) = current; %如果在距离数组里。       
        end 
     else
         if distanceFromStart(neighborIndex(i))> min_dist + 1  
          distanceFromStart(neighborIndex(i)) = min_dist+1; 
             parent(neighborIndex(i)) = current; %如果在距离数组里。 
         end
     end
  end 
 end 
%  for i=1:length(neighborIndex2) 
%  if (map1(neighborIndex2(i))~=2) && (map1(neighborIndex2(i))~=3 && map1(neighborIndex2(i))~= 5) 
%      map1(neighborIndex2(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
%    if distanceFromgoal(neighborIndex2(i))> min_dist1 + 1  
%        distanceFromgoal(neighborIndex2(i)) = min_dist1+1; 
%          parent(neighborIndex2(i)) = current1; %如果在距离数组里，。       
%    end 
%   end 
%  end 
%pause(1);
 end
%  if(map1(neighborIndex)~= 2)
%  %if(map1(neighborIndex1)~= 2 )
%   for i=1:length(neighborIndex1) 
%    if (map1(neighborIndex1(i))~=2) && (map1(neighborIndex1(i))~=3 && map1(neighborIndex1(i))~= 5 )
%      map1(neighborIndex1(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
%     if distanceFromStart(neighborIndex1(i))> min_dist + sqrt(2) 
%        distanceFromStart(neighborIndex1(i)) = min_dist+ sqrt(2); 
%          parent(neighborIndex1(i)) = current; %如果在距离数组里，。
%     end
%    end
%   end
%  end
%  end
   frame = getframe;
    writeVideo(writerObj,frame);
 %
if (isinf(distanceFromStart(dest_node))) 
    route = [];
else
    %提取路线坐标
  route =dest_node ;
  while (parent(route(1)) ~= 0) 
         route = [parent(route(1)), route];     
   end 
%  动态显示出路线 
        for k = 2:length(route) - 1 
              map1(route(k)) = 6; 
               image(1.5, 1.5, map1);
              grid on; 
              axis image; 
               frame = getframe;
    writeVideo(writerObj,frame);
        end  
         b=min_dist
end        
close(writerObj);
title('基于{ \color{red}Dijkstra} 算法的路径规划 ','fontsize',16)
toc