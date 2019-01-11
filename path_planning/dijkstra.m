clc;
clear;
close all;
 uicontrol('Style','pushbutton','String','again', 'FontSize',12, ...
       'Position', [1 1 60 40], 'Callback','dijkstra');
%% set up color map for display 
cmap = [1 1 1; ...%  1 - white - �յ�
        0 0 0; ...% 2 - black - �ϰ� 
        1 0 0; ...% 3 - red - ���������ĵط�
        0 0 1; ...% 4 - blue - �´�������ѡ���� 
        0 1 0; ...% 5 - green - ��ʼ��
        1 1 0;...% 6 - yellow -  ��Ŀ����·�� 
       1 0 1];% 7 - -  Ŀ��� 
colormap(cmap); 
map1 = zeros(10); 
wallpercent=0.4;
% % �������ϰ� 
map1(1:5, 7) = 2;
map1(8,1:3) = 2; 
map1(2:5,3:5)=2;
%map1(ceil(10^2.*rand(floor(10*10*wallpercent),1))) =2;
%  map(ceil(10.*rand),ceil(10.*rand)) = 5; % ��ʼ��
%map(ceil(10.*rand),ceil(10.*rand)) = 6; % Ŀ���
% %% ������ͼ
nrows = 10; 
ncols = 10;  
start_node = sub2ind(size(map1), 10,1); 
dest_node = sub2ind(size(map1),1,4); 
map1(dest_node) = 7;
% % ���������ʼ��
distanceFromStart = Inf(nrows,ncols);  
distanceFromStart(start_node) = 0; 
distanceFromgoal = Inf(nrows,ncols);  
distanceFromgoal(dest_node) = 0; 
% % ����ÿ����Ԫ��������鱣���丸�ڵ�������� 
parent = zeros(nrows,ncols); 
% % ��ѭ��
writerObj = VideoWriter('Dijkstra.avi');
open(writerObj);
tic
 while true 
  % ������״ͼ
  map1(start_node) = 5;
  map1(dest_node) = 7;
  image(1.5, 1.5, map1); 
  grid on; 
  axis image; 
  drawnow; 
   % �ҵ�������ʼ������Ľڵ�
  [min_dist, current] = min(distanceFromStart(:)); %���ص�ǰ��������(������㣩����Сֵ����Сֵ��λ��������
  %[min_dist1, current1] = min(distanceFromgoal(:)); %���ص�ǰ�������飨����Ŀ��㣩����Сֵ����Сֵ��λ��������
   if ((current == dest_node) || isinf(min_dist)) %������Ŀ������ȫ�������꣬����ѭ����
         break; 
   end; 
% if((current==current1)|| isinf(min_dist)|| isinf(min_dist1))
%       break;
% end      
 map1(current) = 3; %����ǰ��ɫ��Ϊ��ɫ��
 %map1(current1)=3;
distanceFromStart(current) = Inf;  %��ǰ�����ھ�������������Ϊ�����ʾ��������
%distanceFromgoal(current1) = Inf;  %��ǰ�����ھ�������������Ϊ�����ʾ��������
 [i, j] = ind2sub(size(distanceFromStart), current); %���ص�ǰλ�õ�����
 %[i1, j1] = ind2sub(size(distanceFromgoal), current1); %���ص�ǰλ�õ�����
 neighbor = [ 
            i-1,j;... 
            i+1,j;... 
            i,j+1;... 
             i,j-1]; %ȷ����ǰλ�õ�������������
% neighbor2 = [ 
%             i1-1,j1;... 
%             i1+1,j1;... 
%             i1,j1+1;... 
%              i1,j1-1]; %ȷ����ǰλ�õ�������������        
      neighbor1 = [ 
              i-1,j-1;...
              i+1,j+1;...
              i-1,j+1;...
              i+1,j-1]; %ȷ����ǰλ�õĶԽ�����      
 outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) +...
                    (neighbor(:,2)<1) + (neighbor(:,2)>ncols ); %�ж���һ�������������Ƿ񳬳����ơ�   
  outRangetest1 = (neighbor1(:,1)<1) + (neighbor1(:,1)>nrows) +...
                     (neighbor1(:,2)<1) + (neighbor1(:,2)>ncols ); %�ж���һ�������������Ƿ񳬳����ơ�        
% outRangetest2 = (neighbor2(:,1)<1) + (neighbor2(:,1)>nrows) +...
%                    (neighbor2(:,2)<1) + (neighbor2(:,2)>ncols ); %�ж���һ�������������Ƿ񳬳����ơ�
 locate = find(outRangetest>0); %���س��޵��������
   locate1 = find(outRangetest1>0); %���س��޵��������
 %locate2 = find(outRangetest2>0); %���س��޵��������
 neighbor(locate,:)=[]; %����һ������������ȥ�����޵㣬ɾ��ĳһ�С�
 neighborIndex = sub2ind(size(map1),neighbor(:,1),neighbor(:,2)); %�����´���������������š�
   neighbor1(locate1,:)=[]; %����һ������������ȥ�����޵㣬ɾ��ĳһ�С�
    neighborIndex1 = sub2ind(size(map1),neighbor1(:,1),neighbor1(:,2)); %�����´���������������š�
%     neighbor2(locate2,:)=[]; %����һ������������ȥ�����޵㣬ɾ��ĳһ�С�
%     neighborIndex2 = sub2ind(size(map1),neighbor2(:,1),neighbor2(:,2)); %�����´���������������š�

 for i=1:length(neighborIndex) 
 if (map1(neighborIndex(i))~=2) && (map1(neighborIndex(i))~=3 && map1(neighborIndex(i))~= 5) 
     map1(neighborIndex(i)) = 4; %����´������ĵ㲻���ϰ���������㣬û���������ͱ�Ϊ��ɫ��
     if((neighborIndex(i)+1==current)||(neighborIndex(i)-1==current))
        if distanceFromStart(neighborIndex(i))> min_dist + 2  
          distanceFromStart(neighborIndex(i)) = min_dist+2; 
             parent(neighborIndex(i)) = current; %����ھ��������       
        end 
     else
         if distanceFromStart(neighborIndex(i))> min_dist + 1  
          distanceFromStart(neighborIndex(i)) = min_dist+1; 
             parent(neighborIndex(i)) = current; %����ھ�������� 
         end
     end
  end 
 end 
%  for i=1:length(neighborIndex2) 
%  if (map1(neighborIndex2(i))~=2) && (map1(neighborIndex2(i))~=3 && map1(neighborIndex2(i))~= 5) 
%      map1(neighborIndex2(i)) = 4; %����´������ĵ㲻���ϰ���������㣬û���������ͱ�Ϊ��ɫ��
%    if distanceFromgoal(neighborIndex2(i))> min_dist1 + 1  
%        distanceFromgoal(neighborIndex2(i)) = min_dist1+1; 
%          parent(neighborIndex2(i)) = current1; %����ھ����������       
%    end 
%   end 
%  end 
%pause(1);
 end
%  if(map1(neighborIndex)~= 2)
%  %if(map1(neighborIndex1)~= 2 )
%   for i=1:length(neighborIndex1) 
%    if (map1(neighborIndex1(i))~=2) && (map1(neighborIndex1(i))~=3 && map1(neighborIndex1(i))~= 5 )
%      map1(neighborIndex1(i)) = 4; %����´������ĵ㲻���ϰ���������㣬û���������ͱ�Ϊ��ɫ��
%     if distanceFromStart(neighborIndex1(i))> min_dist + sqrt(2) 
%        distanceFromStart(neighborIndex1(i)) = min_dist+ sqrt(2); 
%          parent(neighborIndex1(i)) = current; %����ھ����������
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
    %��ȡ·������
  route =dest_node ;
  while (parent(route(1)) ~= 0) 
         route = [parent(route(1)), route];     
   end 
%  ��̬��ʾ��·�� 
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
title('����{ \color{red}Dijkstra} �㷨��·���滮 ','fontsize',16)
toc