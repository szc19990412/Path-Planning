% P=imread('1.jpg');%在指定路径下存原图片
% I = rgb2gray(P);%获得灰度图像

I=imread('map1.bmp');
% a=100;
% b=100;
% l=1;%栅格大小
a=100;
b=100;
l=1;%栅格大小
B = imresize(I,[a/l b/l]);%转化为指定像素大小
J=floor(B/255);
longitude=0:a;
latitude=0:b; 
%坐标轴控制
axes('GridLineStyle', '-');
set(gca,'ydir','reverse');
%set(gca,'xdir','reverse')
hold on
axis([0,a,0,b]);
set(gca,'xtick',0:10:a,'ytick',0:10:b);
set(gca,'xtick',longitude,'ytick',latitude)
grid on 
for i=1:a/l-1
for j=1:b/l-1
if(J(i,j)==0)
y=[i,i,i+1,i+1]*l;
x=[j,j+1,j+1,j]*l;
h=fill(x,y,'r');
hold on
end
end
end