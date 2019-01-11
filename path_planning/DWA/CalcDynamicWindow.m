function Vr=CalcDynamicWindow(x,model)
%
global dt;
% 车子速度的最大最小范围
Vs=[0 model(1) -model(2) model(2)];
 
% 根据当前速度以及加速度限制计算的动态窗口
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
 
% 最终的Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];