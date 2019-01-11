close all;
disp('Dynamic Window Approach sample program start!!')
x=[0 0 pi/2 0 0]';% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
goal=[10,10];% 目标点位置 [x(m),y(m)]
% 障碍物位置列表 [x(m) y(m)]
obstacle=[0 2;
          4 2;
          4 4;
          5 4;
          5 5;
          5 6;
          5 9
          8 8
          8 9
          7 9
          6 5
          6 3
          6 8
          6 7
          7 4
          9 8
          9 11
          9 6];
      
obstacleR=0.5;% 冲突判定用的障碍物半径
global dt; dt=0.1;% 时间[s]
% 机器人运动学模型
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];
% 评价函数参数 [heading,dist,velocity,predictDT]
evalParam=[0.05,0.2,0.1,3.0];
area=[-1 11 -1 11];% 模拟区域范围 [xmin xmax ymin ymax]
% 模拟实验的结果
result.x=[];
tic;
% movcount=0;
% Main loop
for i=1:5000
    % DWA参数输入
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);% 机器人移动到下一个时刻
    % 模拟结果的保存
    result.x=[result.x; x'];
    % 是否到达目的地
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end   
    %====Animation====
    hold off;
    ArrowLength=0.5;% 
    % 机器人
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(goal(1),goal(2),'*r');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    % 探索轨迹
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
end
toc

% function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% % Dynamic Window [vmin,vmax,wmin,wmax]
% Vr=CalcDynamicWindow(x,model);
% % 评价函数的计算
% [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);
% if isempty(evalDB)
%     disp('no path to goal!!');
%     u=[0;0];return;
% end
% % 各评价函数正则化
% evalDB=NormalizeEval(evalDB);
% % 最终评价函数的计算
% feval=[];
% for id=1:length(evalDB(:,1))
%     feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
% end
% evalDB=[evalDB feval];
%  
% [maxv,ind]=max(feval);% 最优评价函数
% u=evalDB(ind,1:2)';% 
% function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
% % 
% evalDB=[];
% trajDB=[];
% for vt=Vr(1):model(5):Vr(2)
%     for ot=Vr(3):model(6):Vr(4)
%         % 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹
%         [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
%         % 各评价函数的计算
%         heading=CalcHeadingEval(xt,goal);
%         dist=CalcDistEval(xt,ob,R);
%         vel=abs(vt);
%         % 制动距离的计算
%         stopDist=CalcBreakingDist(vel,model);
%         if dist>stopDist % 
%             evalDB=[evalDB;[vt ot heading dist vel]];
%             trajDB=[trajDB;traj];
%         end
%     end
% end
% function EvalDB=NormalizeEval(EvalDB)
% % 评价函数正则化
% if sum(EvalDB(:,3))~=0
%     EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
% end
% if sum(EvalDB(:,4))~=0
%     EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
% end
% if sum(EvalDB(:,5))~=0
%     EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
% end
% function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% % 轨迹生成函数
% % evaldt：前向模拟时间; vt、ot当前速度和角速度; 
% global dt;
% time=0;
% u=[vt;ot];% 输入值
% traj=x;% 机器人轨迹
% while time<=evaldt
%     time=time+dt;% 时间更新
%     x=f(x,u);% 运动更新
%     traj=[traj x];
% end
% function stopDist=CalcBreakingDist(vel,model)
% % 根据运动学模型计算制动距离,这个制动距离并没有考虑旋转速度，不精确吧！！！
% global dt;
% stopDist=0;
% while vel>0
%     stopDist=stopDist+vel*dt;% 制动距离的计算
%     vel=vel-model(3)*dt;% 
% end
% function dist=CalcDistEval(x,ob,R)
% % 障碍物距离评价函数
% dist=100;
% for io=1:length(ob(:,1))
%     disttmp=norm(ob(io,:)-x(1:2)')-R;%僷僗偺埵抲偲忈奞暔偲偺僲儖儉岆嵎傪寁嶼
%     if dist>disttmp% 离障碍物最小的距离
%         dist=disttmp;
%     end
% end
%  
% % 障碍物距离评价限定一个最大值，如果不设定，一旦一条轨迹没有障碍物，将太占比重
% if dist>=2*R
%     dist=2*R;
% end
%  
% function heading=CalcHeadingEval(x,goal)
% % heading的评价函数计算
%  
% theta=toDegree(x(3));% 机器人朝向
% goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点的方位
%  
% if goalTheta>theta
%     targetTheta=goalTheta-theta;% [deg]
% else
%     targetTheta=theta-goalTheta;% [deg]
% end
%  
% heading=180-targetTheta;
%  
% function Vr=CalcDynamicWindow(x,model)
% %
% global dt;
% % 车子速度的最大最小范围
% Vs=[0 model(1) -model(2) model(2)];
%  
% % 根据当前速度以及加速度限制计算的动态窗口
% Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
%  
% % 最终的Dynamic Window
% Vtmp=[Vs;Vd];
% Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%  
% function x = f(x, u)
% % Motion Model
% % u = [vt; wt];当前时刻的速度、角速度
% global dt;
%  
% F = [1 0 0 0 0
%      0 1 0 0 0
%      0 0 1 0 0
%      0 0 0 0 0
%      0 0 0 0 0];
%  
% B = [dt*cos(x(3)) 0
%     dt*sin(x(3)) 0
%     0 dt
%     1 0
%     0 1];
%  
% x= F*x+B*u;
%  
% function radian = toRadian(degree)角度转弧度
% % degree to radian
% radian = degree/180*pi;
%  
% function degree = toDegree(radian)弧度转角度
% % radian to degree
% degree = radian/pi*180;
