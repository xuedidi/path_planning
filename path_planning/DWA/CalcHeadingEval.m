function heading=CalcHeadingEval(x,goal)
% heading的评价函数计算
 
theta=toDegree(x(3));% 机器人朝向
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点的方位
 
if goalTheta>theta
    targetTheta=goalTheta-theta;% [deg]
else
    targetTheta=theta-goalTheta;% [deg]
end
 
heading=180-targetTheta;