%% 清空环境
clc;
clear all;
close all;
tic
%% 障碍物数据
position = load('barrier.txt');
% plot([0,100],[0,100],'.');
axis([0 100 0 100]);
hold on
B = load('barrier.txt');
xlabel('km','fontsize',12)
ylabel('km','fontsize',12)
title('二维规划空间','fontsize',12)
%% 描述起点和终点
Start = [10,90];
Target = [80,45];
plot([Start(1),Target(1)],[Start(2),Target(2)],'.');

% 图形标注
text(Start(1),Start(2)+7,'S');
text(Target(1)+3,Target(2)-4,'T');
 
%% 描绘障碍物图形
fill(position(1:4,1),position(1:4,2),[0,0,0]);
fill(position(5:8,1),position(5:8,2),[0,0,0]);
fill(position(9:12,1),position(9:12,2),[0,0,0]);
fill(position(13:15,1),position(13:15,2),[0,0,0]);

% 下载链路端点数据
L = load('lines.txt');
 
%% 描绘线及中点
v = zeros(size(L)); %存储中点的坐标
for i=1:20   %%20表示连线数目即中点数目
    plot([position(L(i,1),1),position(L(i,2),1)],[position(L(i,1),2)...
        ,position(L(i,2),2)],'color','black','LineStyle','--');
    v(i,:) = (position(L(i,1),:)+position(L(i,2),:))/2;
    plot(v(i,1),v(i,2),'*');
    text(v(i,1)+2,v(i,2),strcat('v',num2str(i)));
end

%% 描绘可行路径
sign = load('matrix.txt');%邻接矩阵
[n,m]=size(sign);

for i=1:n
    
    if i == 1
        for k=1:m-1
            if sign(i,k) == 1
                plot([Start(1),v(k-1,1)],[Start(2),v(k-1,2)],'color',...
                    'black','Linewidth',1,'LineStyle','-');
            end
        end
        continue;
    end
    
    for j=2:i
        if i == m
            if sign(i,j) == 1
                plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color',...
                    'black','Linewidth',1,'LineStyle','-');
            end
        else
            if sign(i,j) == 1
                plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],...
                    'color','black','Linewidth',1,'LineStyle','-');
            end
        end
    end
end                                                                  %到此能画出MAKLINK链路图
v1=zeros(22,2);  %20个中点加上起始点和终止点                         %v1存放包括S和T的总共22个结点，作为DijkstraPlan的参数
v1(1,:)=[20,180];
v1(22,:)=[160,90];
for i=2:21
    v1(i,:)=v(i-1,:);
end
path = DijkstraPlan(v1,sign);
j = path(22);
plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color','black','LineWidth',3,'LineStyle',':');
i = path(22);
j = path(i);
count = 0;
while true
    plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],'color','black','LineWidth',3,'LineStyle',':');
    count = count + 1;
    i = j;
    j = path(i);
    if i == 1 || j==1
        break;
    end
end
plot([Start(1),v(i-1,1)],[Start(2),v(i-1,2)],'color','black','LineWidth',3,'LineStyle',':');
count = count+3;
pathtemp(count) = 22;
j = 22;
for i=2:count
    pathtemp(count-i+1) = path(j);
    j = path(j);
end
path = pathtemp;                     %到此能在规划空间中用黄线描绘出次优最短路径
pathCount = length(path)-2;          %经过线段数量，相当于维数D

%% 经过的链接线
lines = zeros(pathCount,4);
for i = 1:pathCount                  %lines用于存放各个结点所在链接线的起点和终点
    lines(i,1:2) = B(L(path(i+1)-1,1),:);
    lines(i,3:4) = B(L(path(i+1)-1,2),:);
end

% 初始最短路径
dijpathlen = 0;
vv = zeros(22,2);
vv(1,:) = Start;
vv(22,:) = Target;
vv(2:21,:) = v;
for i=1:pathCount+1
dijpathlen = dijpathlen + sqrt((vv(path(i),1)-vv(path(i+1),1))^2+(vv(path(i),2)-vv(path(i+1),2))^2);
end
LL = dijpathlen;                   %得出的LL为次优最短路径的长度

%% 果蝇优化算法参数初始化
popsize=50;
maxgen=200;
gen=0;
D=zeros(popsize,pathCount);
S=zeros(popsize,pathCount);
bestgensmell=zeros(1,maxgen);

%% 果蝇位置初始化及食物浓度计算
%*** 随机初始果蝇群体位置。
X_axis=1+1*rands(1,pathCount);
Y_axis=1+1*rands(1,pathCount);

% *** 果蝇寻优开始，利用嗅觉寻找食物。
for p=1:popsize
    X(p,:)=X_axis+2*rand()-1;
    Y(p,:)=Y_axis+2*rand()-1;

    for i=1:pathCount
    %*** 求出与原点之距离
%     D(p,i)=(X(p,i)^2+Y(p,i)^2)^0.5;
%     %*** 味道浓度为距离之倒数，先求出味道浓度判定值。
%     S(p,i)=1/D(p,i);
    S(p,i)=(X(p,i)+Y(p,i))*1.0/2;
    %%%%%%
    if(S(p,i)>1)%%范围限定在0到1之间
            S(p,i)=1;
    end
        if(S(p,i)<0)
            S(p,i)=0;
        end

    end
    
end


    %*** 利用味道浓度判定函数求出味道浓度
for p=1:popsize
    w=S(p,:);
    Smell(p)=distance(w,pathCount,lines,Start,Target);
end

%% 初始化公告板
[bestsmell, bestindex]=min(Smell);
%*** 利用视觉寻找伙伴聚集味道浓度最高之处，做法是保留最佳值初始位置及初始味道浓度。
X_axis=X(bestindex,:);
Y_axis=Y(bestindex,:);
bestS=S(bestindex,:);
SmellBest=bestsmell;

%% 果蝇搜索过程
%*** 果蝇迭代寻优
for gen=1:maxgen
    %*** 利用嗅觉寻找食物
    for p=1:popsize
    %*** 初始果蝇个体飞行距离
    X(p,:)=X_axis+2*rand()-1;
    Y(p,:)=Y_axis+2*rand()-1;

    for i=1:pathCount
    %*** 求出与原点之距离
%     D(p,i)=(X(p,i)^2+Y(p,i)^2)^0.5;
%     %*** 味道浓度为距离之倒数，先求出味道浓度判定值。
%     S(p,i)=1/D(p,i);
    S(p,i)=(X(p,i)+Y(p,i))*1.0/2;
    %%%%%%
    if(S(p,i)>1)%%范围限定在0到1之间
            S(p,i)=1;
    end
        if(S(p,i)<0)
            S(p,i)=0;
        end
    end
    
    end

    %*** 利用味道浓度判定函数求出味道浓度
    for p=1:popsize
        w=S(p,:);
        Smell(p)=distance(w,pathCount,lines,Start,Target);
    end
    [bestsmell, bestindex]=min(Smell);
    
    %*** 利用视觉寻找伙伴聚集味道浓度最高之处，做法是保留最佳值初始位置及初始味道浓度
    if bestsmell<SmellBest
    X_axis=X(bestindex,:);
    Y_axis=Y(bestindex,:);
    bestS=S(bestindex,:);
    SmellBest=bestsmell;
    end
    bestgensmell(1,gen)=SmellBest;
    yy(gen)=SmellBest;
    Xbest(gen,:)=X_axis;
    Ybest(gen,:)=Y_axis;
end
t_train=toc;             %结束记时
%画出公告板中的最短路径轨迹
lastline=zeros(pathCount+2,2);
lastline(1,:)=Start;
lastline(pathCount+2,:)=Target;
pathkbest=bestS;
for i=1:pathCount
    lastline(i+1,:)=lines(i,1:2) + (lines(i,3:4)-lines(i,1:2))*pathkbest(:,i);
end

for i=1:pathCount+1
    plot([lastline(i,1),lastline(i+1,1)],[lastline(i,2),lastline(i+1,2)],'color','black','LineWidth',3);
end
%画出最路径随迭代次数的变化
% figure(2);
% % plot(bestgensmell,'color','red');
% plot(bestgensmell);
% hold on
% title( ['本次运行得到的最优值为',num2str(SmellBest),',总共耗时',num2str(t_train),'s'] );
% %text(180,190,'最优值随迭代次数变化曲线');
% ylabel('路径总长度');
% xlabel('迭代次数');
% figure(3);
% plot(Xbest,Ybest,'b.');
% title('Fruit fly flying route','fontsize',14)
% xlabel('X-axis','fontsize',12);ylabel('Y-axis','fontsize',12);
