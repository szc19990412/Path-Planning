clc,clear all 
%初始化车的参数
Xo=[0 0];%起点位置
k=15;%计算引力需要的增益系数
K=0;%初始化
m=16;%计算斥力的增益系数，都是自己设定的。
Po=15;%障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响。也是自己设定。
d_goal = 80;%引力场影响距离的一个阈值
b_goal = 15;%克服目标点振荡
n=15;%障碍个数
a=5;
l=2;%步长
J=200;%循环迭代次数
%如果不能实现预期目标，可能也与初始的增益系数，Po设置的不合适有关。
%end
%给出障碍和目标信息
% a=10;n=15;%自己改
p=unifrnd(0,100,n,2);%n个长a的正方形中的随机点的坐标，均匀分布

X_rand = p(:,1);
Y_rand = p(:,2);
Xsum=[100 100;X_rand(1) Y_rand(1);X_rand(2) Y_rand(2);X_rand(3) Y_rand(3);X_rand(4) Y_rand(4);X_rand(5) Y_rand(5);X_rand(6) Y_rand(6);X_rand(7) Y_rand(7);...
    X_rand(8) Y_rand(8);X_rand(9) Y_rand(9);X_rand(10) Y_rand(10);X_rand(11) Y_rand(11);X_rand(12) Y_rand(12);X_rand(13) Y_rand(13);...
    X_rand(14) Y_rand(14);X_rand(15) Y_rand(15)];%这个向量是(n+1)*2维，其中[10 10]是目标位置，剩下的都是障碍的位置。
Xj=Xo;%j=1循环初始，将车的起始坐标赋给Xj
%***************初始化结束，开始主体循环******************
for j=1:J %循环开始
Goal(j,1)=Xj(1); %Goal是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
Goal(j,2)=Xj(2);
%调用计算角度模块
Theta=compute_angle(Xj,Xsum,n);%Theta是计算出来的车和障碍，和目标之间的与X轴之间的夹角，统一规定角度为逆时针方向，用这个模块可以计算出来。

%调用计算引力模块
Angle=Theta(1);%Theta（1）是车和目标之间的角度，目标对车是引力。
angle_at=Theta(1);%为了后续计算斥力在引力方向的分量赋值给angle_at
[Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle,b_goal,d_goal,n); %计算出目标对车的引力在x,y方向的两个分量值。
for i=1:n
angle_re(i)=Theta(i+1);%计算斥力用的角度，是个向量，因为有n个障碍，就有n个角度。
end

%调用计算斥力模块
[Frerxx,Freryy,Fataxx,Fatayy]=compute_repulsion(Xj,Xsum,m,angle_at,angle_re,n,Po,a);%计算出斥力在x,y方向的分量数组。
%计算合力和方向，这有问题，应该是数，每个j循环的时候合力的大小应该是一个唯一的数，不是数组。应该把斥力的所有分量相加，引力所有分量相加。
Fsumyj=Faty-Freryy+Fatayy;%y方向的合力
Fsumxj=Fatx-Frerxx+Fataxx;%x方向的合力
Position_angle(j)=atan(Fsumyj/Fsumxj);%合力与x轴方向的夹角向量

% 计算车的下一步位置
if (((Position_angle(j)>0.7)&&(Fsumyj<-0&&Fsumxj<-0))||(Fsumyj<0&&Fsumxj>0))%障碍在上方
    Xnext(1)=Xj(1)+l*cos(Position_angle(j)-pi/4);
    Xnext(2)=Xj(2)+l*sin(Position_angle(j)-pi/4);
 elseif ((Position_angle(j)<0.7)&&(Fsumyj<-0&&Fsumxj<-0))||(Fsumyj<0&&Fsumxj>0)%障碍在下方
    Xnext(1)=Xj(1)+l*cos(Position_angle(j)+pi/4);
    Xnext(2)=Xj(2)+l*sin(Position_angle(j)+pi/4);  
else    
    Xnext(1)=Xj(1)+l*cos(Position_angle(j));
    Xnext(2)=Xj(2)+l*sin(Position_angle(j));
end
%保存车的每一个位置在向量中
Xj=Xnext;
%判断
if ((Xj(1)-Xsum(1,1))>0)&&((Xj(2)-Xsum(1,2))>0) %是应该完全相等的时候算作到达，还是只是接近就可以
    K=j;%记录迭代到多少次，到达目标。
break;
%记录此时的j值
end%如果不符合if的条件，重新返回循环，继续执行。
end%大循环结束

K=j;
Goal(K,1)=Xsum(1,1);%把路径向量的最后一个点赋值为目标
Goal(K,2)=Xsum(1,2);

%***********************************画出障碍，起点，目标，路径点*************************
%画出路径
X=Goal(:,1);
Y=Goal(:,2);
%路径向量Goal是二维数组,X,Y分别是数组的x,y元素的集合，是两个一维数组。
x=X_rand;%障碍的xd坐标
y=Y_rand;

G_size = size(Goal,1);
fmat=moviein(G_size);
for j=1:G_size
plot(100,100,'v',0,0,'ms',X(1:j),Y(1:j),'.r');
axis([-5,105,-5,105]);
set(gca,'xtick',(-5:1:105))
set(gca,'ytick',(-5:1:105))
grid on 
hold on
for k=1:2:15
fill([x(k)-1 x(k)-1 x(k)+1 x(k)+1],[y(k)-1 y(k)+1 y(k)+1 y(k)-1],'k');
end
for k=2:2:14
fill([x(k)-2 x(k)-2 x(k)+2 x(k)+2],[y(k)-2 y(k)+2 y(k)+2 y(k)-2],'k');
end
fmat(:,j)=getframe;
end
movie(fmat,10)