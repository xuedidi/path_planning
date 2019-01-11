function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% Dynamic Window [vmin,vmax,wmin,wmax]
Vr=CalcDynamicWindow(x,model);
% 评价函数的计算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);
if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end
% 各评价函数正则化
evalDB=NormalizeEval(evalDB);
% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];
 
[maxv,ind]=max(feval);% 最优评价函数
u=evalDB(ind,1:2)';% 