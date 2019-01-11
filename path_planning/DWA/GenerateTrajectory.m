function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% 轨迹生成函数
% evaldt：前向模拟时间; vt、ot当前速度和角速度; 
global dt;
time=0;
u=[vt;ot];% 输入值
traj=x;% 机器人轨迹
while time<=evaldt
    time=time+dt;% 时间更新
    x=f(x,u);% 运动更新
    traj=[traj x];
end