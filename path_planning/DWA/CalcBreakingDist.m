function stopDist=CalcBreakingDist(vel,model)
% 根据运动学模型计算制动距离,这个制动距离并没有考虑旋转速度，不精确吧！！！
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;% 制动距离的计算
    vel=vel-model(3)*dt;% 
end