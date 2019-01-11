mapName='map1.bmp'; % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
k=10; % number of points in the PRM
display=true; % display processing of nodes
map=im2bw(imread(mapName),0.5);
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end 
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
hold on
vertex=[source;goal]; % source and goal taken as additional vertices in the path planning to ease planning of the robot 
if display, rectangle('Position',[vertex(1,2)-5,vertex(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
if display, rectangle('Position',[vertex(2,2)-5,vertex(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
tic;
while length(vertex)<k+2 % iteratively add vertices
    x=double(int32(rand(1,2) .* size(map)));
    if feasiblePoint(x,map), 
        vertex=[vertex;x]; 
        if display, rectangle('Position',[x(2)-5,x(1)-5,10,10],'Curvature',[1,1],'FaceColor','r'); end
    end
end
% if display 
%     fprintf('Click to Continue. \n WARNING: The processing may take a minutue or more depending upon the settings\n');
%     waitforbuttonpress;
% end
edges=cell(k+2,1); % edges to be stored as an adjacency list
for i=1:k+2
    for j=i+1:k+2
        if checkPath(vertex(i,:),vertex(j,:),map);
            edges{i}=[edges{i};j];edges{j}=[edges{j};i];
            if display, 
                line([vertex(i,2);vertex(j,2)],[vertex(i,1);vertex(j,1)],'linewidth',1.5);
            end
        end
%         drawnow
    end
   drawnow 
end
hold on;
% if display 
%     fprintf('Click to Continue.\n');
%     waitforbuttonpress; 
% end 
%structure of a node is taken as index of node in vertex, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
Q=[1 0 heuristic(vertex(1,:),goal) 0+heuristic(vertex(1,:),goal) -1]; % the processing queue of A* algorihtm, open list
closed=[]; % the closed list taken as a list
pathFound=false;
while size(Q,1)>0
     [A, I]=min(Q,[],1);
     n=Q(I(4),:); % smallest cost element to process
     Q=[Q(1:I(4)-1,:);Q(I(4)+1:end,:)]; % delete element under processing
     if n(1)==2 % goal test
         pathFound=true;break;
     end
     for mv=1:length(edges{n(1),1}) %iterate through all edges from the node
         newVertex=edges{n(1),1}(mv);
         if length(closed)==0 || length(find(closed(:,1)==newVertex))==0 % not already in closed
             historicCost=n(2)+historic(vertex(n(1),:),vertex(newVertex,:));
             heuristicCost=heuristic(vertex(newVertex,:),goal);
             totalCost=historicCost+heuristicCost;
             add=true; % not already in queue with better cost
             if length(find(Q(:,1)==newVertex))>=1
                 I=find(Q(:,1)==newVertex);
                 if Q(I,4)<totalCost, add=false;
%                  else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                 end
             end
             if add
                 Q=[Q;newVertex historicCost heuristicCost totalCost size(closed,1)+1]; % add new nodes in queue
             end
         end           
     end
     closed=[closed;n]; % update closed lists
end
if ~pathFound
    error('no path found')
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc,n(4)); 
path=[vertex(n(1),:)]; %retrieve path from parent information
prev=n(5);
while prev>0
    path=[vertex(closed(prev,1),:);path];
    prev=closed(prev,5);
end

% clf;
% hold on
line(path(:,2),path(:,1),'color','r','linewidth',3);
