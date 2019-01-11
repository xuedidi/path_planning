function dist=CalcDistEval(x,ob,R)
% 障碍物距离评价函数
dist=100;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%僷僗偺埵抲偲忈奞暔偲偺僲儖儉岆嵎傪寁嶼
    if dist>disttmp% 离障碍物最小的距离
        dist=disttmp;
    end
end
 
% 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
if dist>=2*R
    dist=2*R;
end