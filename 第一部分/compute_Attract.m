function [Yatx,Yaty]=compute_Attract(X,Xsum,k,angle,b_goal,d_goal,n)%输入参数为当前坐标，目标坐标，增益常数,分量和力的角度
%把路径上的临时点作为每个时刻的Xgoal
R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
r=sqrt(R);%路径点和目标的距离

if r>d_goal
    r=d_goal;
elseif r<b_goal
    k=k*20;
    r=b_goal;
else
    k=k*5;    
end

Yatx=k*r*cos(angle);%angle=Y(1)
Yaty=k*r*sin(angle);
end
