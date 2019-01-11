clc;
clear;
figure('name','双向RRT算法')
 uicontrol('Style','pushbutton','String','again', 'FontSize',12, ...
       'Position', [1 1 60 40], 'Callback','astart');
map=im2bw(imread('map3.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 470]; % source position in Y, X format
goal=[420 20]; % goal position in Y, X format
stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 1000;
display=true; % display of RRT
%%%%% parameters end here %%%%%
tic;
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end %检测起点和终点是否可行，检测起点和终点在哪个位置
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end  %edgecolor边框颜色
RRTree1=double([source -1]); % First RRT rooted at the source, representation node and parent index 第一棵树以起点作为根节点
RRTree2=double([goal -1]); % Second RRT rooted at the goal, representation node and parent index  第二颗树以目标点作为根节点
counter=0;
tree1ExpansionFail=false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false; % sets to true if expansion after set number of attempts fails
writerObj = VideoWriter('RRT.avi');
open(writerObj);
while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
         if ~isempty(pathFound)
             tree2ExpansionFail =true;
         end   
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b','linewidth',1.5);
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','m','linewidth',1.5);
            counter=counter+1;M(counter)=getframe;
        end
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green','linewidth',1.5);
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first  第一棵树的路径
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
      frame = getframe;
    writeVideo(writerObj,frame);
end
%% 路径优化
%path1=flipud(sub2ind(size(map),path(:,1),path(:,2)));
path1=flipud(path);
path_smooth =path1(end,:);
tmp_point = path1(end,:);
while true
    l_p = length(path1);
     for i=1:l_p -1
         if checkPath(path1(i,:), tmp_point, map)
             path_smooth = [path_smooth; path1(i,:)];
              tmp_point = path1(i,:);
              break;
         else
                continue;
         end
     end   
     if(tmp_point==path1(1,:))
         break;
     end
      if checkPath(path1(1,:), tmp_point, map)
          path_smooth = [path_smooth; path1(1,:)];
          break;
      else
           path1=path1(1:i,:);
     end
end         
if display 
    disp('click/press any key'); 
    waitforbuttonpress; 
end
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
for i=1:length(path_smooth)-1, pathLength=pathLength+distanceCost( path_smooth(i,1:2), path_smooth(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n', toc,pathLength); 
% imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1),'color','r','linewidth',3);
line(path_smooth(:,2),path_smooth(:,1),'color','g','linewidth',3);
frame = getframe;
writeVideo(writerObj,frame);
close(writerObj);
title('双向快速搜索随机树(Rapidly-Explore Random Trees)','color','red')