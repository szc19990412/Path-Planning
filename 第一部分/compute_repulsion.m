function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(X,Xsum,m,angle_at,angle_re,n,Po,a)%输入参数为当前坐标，Xsum是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
Rat=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
rat=sqrt(Rat);%路径点和目标的距离
for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%路径点和障碍的距离平方
    rre(i)=sqrt(Rrei(i));%路径点和障碍的距离保存在数组rrei中
    R0=(Xsum(1,1)-Xsum(i+1,1))^2+(Xsum(1,2)-Xsum(i+1,2))^2;
    r0=sqrt(R0);
    if rre(i)>Po || rat<15%如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
        Yrerx(i)=0;
        Yrery(i)=0;
        Yatax(i)=0;
        Yatay(i)=0;
    else
    if rre(i)<Po/2
        Yrer(i)=m*(1/rre(i)-1/Po)*(1/Rrei(i))*(rat^a);%分解的Fre1向量,斥力方向
        Yata(i)=a*m*((1/rre(i)-1/Po)^2)*(rat^(1-a))/2;%分解的Fre2向量,引力方向
%         Yrerx(i)=(1+0.1)*Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
%         Yrery(i)=-(1-0.1)*Yrer(i)*sin(angle_re(i));
        Yrerx(i)=Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
        Yrery(i)=Yrer(i)*sin(angle_re(i));
        Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
        Yatay(i)=Yata(i)*sin(angle_at);
    else
        Yrer(i)=m*(1/rre(i)-1/Po)*1/Rrei(i)*Rat;%分解的Fre1向量
        Yata(i)=m*((1/rre(i)-1/Po)^2)*rat;%分解的Fre2向量
        Yrerx(i)=Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
        Yrery(i)=Yrer(i)*sin(angle_re(i));
        Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
        Yatay(i)=Yata(i)*sin(angle_at);
    end%判断距离是否在障碍影响范围内
    end
end
    Yrerxx=sum(Yrerx);%叠加斥力的分量
    Yreryy=sum(Yrery);
    Yataxx=sum(Yatax);
    Yatayy=sum(Yatay);
end

