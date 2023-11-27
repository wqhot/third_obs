clear all
close all
clc
%模型变量调整
[L1,L2,L3]=deal(5,3,2);%目标的外包络，长、宽、高，单位：m
[Xo,Yo,Ho]=deal(0,0,0);%观察者墨卡托坐标，单位：m
[Xt,Yt,Ht]=deal(0,20,0);%目标墨卡托坐标，单位：m
[phi_o,theta_o,psi_o]=deal(0/180*pi,0/180*pi,0/180*pi);%观察者载体坐标系相对于水平坐标系的姿态
[phi_t,theta_t,psi_t]=deal(45/180*pi,0/180*pi,0/180*pi);%目标载体坐标系相对于水平坐标系的姿态
[alpha_o,beta_o]=deal(120/180*pi,0/180*pi);%观察者相机的朝向
[m_o,n_o]=deal(1920,1080);%观察者相机的分辨率

FOV_oh=90/180*pi;%观察者相机水平视场角
FOV_ov=2*atan(n_o/m_o*tan(FOV_oh/2));%观察者相机垂直视场角

FOV_hl=45/180*pi;%观察者相机水平视场角左边，这里考虑了光轴不过CCD靶面中心的情况
FOV_hr=45/180*pi;%观察者相机水平视场角右边，这里考虑了光轴不过CCD靶面中心的情况
FOV_vu=atan(n_o/m_o*tan(FOV_hl));%观察者相机垂直视场角上边，这里考虑了光轴不过CCD靶面中心的情况
FOV_vd=atan(n_o/m_o*tan(FOV_hr));%观察者相机垂直视场角下边，这里考虑了光轴不过CCD靶面中心的情况

%视场角与相机内参的转换关系
f_x=m_o/(tan(FOV_hl)+tan(FOV_hr));
c_x=m_o*tan(FOV_hl)/(tan(FOV_hl)+tan(FOV_hr));
f_y=n_o/(tan(FOV_vu)+tan(FOV_vd));
c_y=n_o*tan(FOV_vu)/(tan(FOV_vu)+tan(FOV_vd));

method=1;%仿真中既实现了视场角的方法，也实现了相机内参的方法，0为视场角 1为相机内参
T_of_ob=[0;0;0];%相机在载体坐标系中的安装位置
T_oc_of=[0;0;0];%相机在相机安装坐标系中的平移

% X_t_tb=[0;0;0]
X_t_tb=[0 -L2 L2 L2 -L2 -L2 L2 L2 -L2;
        0 L1 L1 L1 L1 -L1 -L1 -L1 -L1;
        0 L3 L3 -L3 -L3 L3 L3 -L3 -L3]/2;%目标载体坐标系中建立需要转换的9个坐标，包含目标中心点和外包络8个顶点
points_num=size(X_t_tb,2);%需要进行坐标转换的坐标数量
    
%需要进行坐标转换的坐标写为齐次坐标的形式
X_t_tb=[X_t_tb; ones(1,points_num)];

%目标载体坐标系中的坐标转换到目标水平坐标系中
R_Y_l_m1=[cos(psi_t) 0 sin(psi_t);0 1 0;-sin(psi_t) 0 cos(psi_t)];
R_X_l_m1=[1 0 0;0 cos(theta_t) -sin(theta_t);0 sin(theta_t) cos(theta_t)];
R_Z_l_m1=[cos(phi_t) -sin(phi_t) 0;sin(phi_t) cos(phi_t) 0;0 0 1];
R_b_l_m1=R_Z_l_m1*R_X_l_m1*R_Y_l_m1;
X_t_tl=[R_b_l_m1 [0;0;0];0 0 0 1]*X_t_tb;

%目标水平坐标系中的坐标转换至观察者水平坐标系中
X_o_web=[Xo;Yo;Ho];
X_t_web=[Xt;Yt;Ht];
X_t_ol=[eye(3) -(X_o_web-X_t_web); 0 0 0 1]*X_t_tl;

%观察者水平坐标系中的坐标转换至观察者载体坐标系
R_Z_ol=[cos(phi_o) sin(phi_o) 0;-sin(phi_o) cos(phi_o) 0;0 0 1];
R_X_ol=[1 0 0;0 cos(theta_o) sin(theta_o);0 -sin(theta_o) cos(theta_o)];
R_Y_ol=[cos(psi_o) 0 -sin(psi_o);0 1 0; sin(psi_o) 0 cos(psi_o)];
R_ob_ol=R_Y_ol*R_X_ol*R_Z_ol;
X_t_ob=[R_ob_ol [0;0;0]; 0 0 0 1]*X_t_ol;

%观察者载体坐标系中的坐标转换至观察者相机安装坐标系中
X_t_of=[eye(3) -T_of_ob; 0 0 0 1]*X_t_ob;

%观察者相机安装坐标系中的坐标转换至观察者相机坐标系中
R_Z_of=[sin(alpha_o) -cos(alpha_o) 0;cos(alpha_o) sin(alpha_o) 0;0 0 1];
R_X_of=[1 0 0;0 sin(beta_o) -cos(beta_o);0 cos(beta_o) sin(beta_o)];
R_oc_of=R_X_of*R_Z_of;
X_t_oc=[R_oc_of -R_oc_of*T_oc_of;0 0 0 1]*X_t_of;

%计算每个坐标的最优角度，即坐标在屏幕中心时相机的朝向
X_c=[eye(3) -T_oc_of;0 0 0 1]*X_t_of;%将坐标中扣除相机的平移，用于相机最优朝向的计算

Alpha_c=zeros(1,points_num);%每个坐标对应的最优α角度，数据存储的单位为：度
Beta_c=zeros(1,points_num);%每个坐标对应的最优β角度，数据存储的单位为：度
for i=1:1:points_num
    if X_c(1,i)>0&&X_c(2,i)>=0
        Alpha_c(i)=180/pi*atan(X_c(2,i)/X_c(1,i));
    elseif X_c(1,i)<=0&&X_c(2,i)>0
        Alpha_c(i)=180/pi*(pi/2+atan(abs(X_c(1,i)/X_c(2,i))));
    elseif X_c(1,i)<0&&X_c(2,i)<=0
        Alpha_c(i)=180/pi*(pi+atan(abs(X_c(2,i)/X_c(1,i))));
    elseif X_c(1,i)>=0&&X_c(2,i)<0
        Alpha_c(i)=180/pi*(3*pi/2+atan(abs(X_c(1,i)/X_c(2,i))));
    else
        Alpha_c(i)=-180;
    end
end

for i=1:1:points_num
    if (X_c(1,i)~=0)||(X_c(2,i)~=0)
        Beta_c(i)=180/pi*atan(X_c(3,i)/sqrt(X_c(1,i)^2+X_c(2,i)^2));
    elseif (X_c(1,i)==0)&&(X_c(2,i)==0)&&(X_c(3,i)>0)
        Beta_c(i)=90;
    elseif (X_c(1,i)==0)&&(X_c(2,i)==0)&&(X_c(3,i)<0)
        Beta_c(i)=-90;
    else
        Beta_c(i)=-180;
    end
end


TARGET_POS_VIEW=zeros(3,points_num);%目标与视野的位置关系标识
%TARGET_POS_VIEW第1行表示对应坐标是否在视野中
%0为无效值 1为不在视野中 2为在视野中
%TARGET_POS_VIEW第2行表示对应坐标水平方向与视野的位置关系
%0为无效值 1为水平方向在视野左侧 2为水平方向在视野中 3为水平方向在视野右侧
%TARGET_POS_VIEW第3行表示对应坐标垂直方向与视野的位置关系
%0为无效值 1为垂直方向在视野上方 2为垂直方向在视野中 3为垂直方向在视野下方
in_view=0;%指示是否有坐标点在视野中 0为不在视野中 1为在视野中

%基于视场角的方法，判断目标与视野的位置关系，并将相机坐标系中的坐标转换至图像像素坐标系中
if method==0
    for i=1:1:points_num
        %水平方向的判断
        if X_t_oc(3,i)<=-1/tan(FOV_hl)*X_t_oc(1,i)&&X_t_oc(1,i)<0
            TARGET_POS_VIEW(2,i)=1;
        elseif X_t_oc(3,i)<=1/tan(FOV_hr)*X_t_oc(1,i)&&X_t_oc(1,i)>0
            TARGET_POS_VIEW(2,i)=3;
        elseif X_t_oc(3,i)>1/tan(FOV_hr)*X_t_oc(1,i)&&X_t_oc(3,i)>-1/tan(FOV_hl)*X_t_oc(1,i)
            TARGET_POS_VIEW(2,i)=2;
        else 
            TARGET_POS_VIEW(2,i)=0;
        end
        
        %垂直方向的判断
        if X_t_oc(3,i)<=-1/tan(FOV_vu)*X_t_oc(2,i)&&X_t_oc(2,i)<0
            TARGET_POS_VIEW(3,i)=1;
        elseif X_t_oc(3,i)<=1/tan(FOV_vd)*X_t_oc(2,i)&&X_t_oc(2,i)>0
            TARGET_POS_VIEW(3,i)=3;
        elseif X_t_oc(3,i)>1/tan(FOV_vd)*X_t_oc(2,i)&&X_t_oc(3,i)>-1/tan(FOV_vu)*X_t_oc(2,i)
            TARGET_POS_VIEW(3,i)=2;
        else
            TARGET_POS_VIEW(3,i)=0;
        end
        
        %判断是否有坐标点在视野中
        if TARGET_POS_VIEW(2:3,i)==[2;2]
            TARGET_POS_VIEW(1,i)=2;
            in_view=1;
        end
    end
        %将相机坐标系中的坐标转换到图像像素坐标系中
        X_t_nor=zeros(3,points_num);%归一化坐标
        %归一化坐标计算
        for i=1:1:points_num
            if X_t_oc(1,i)>=0
                X_t_nor(1,i)=1/tan(FOV_hr)*X_t_oc(1,i)/X_t_oc(3,i);
            else
                X_t_nor(1,i)=1/tan(FOV_hl)*X_t_oc(1,i)/X_t_oc(3,i);
            end

            if X_t_oc(2,i)>=0
                X_t_nor(2,i)=1/tan(FOV_vd)*X_t_oc(2,i)/X_t_oc(3,i);
            else
                X_t_nor(2,i)=1/tan(FOV_vu)*X_t_oc(2,i)/X_t_oc(3,i);
            end
            X_t_nor(3,i)=1;
        end
        %归一化坐标到像素坐标
        S_p_nor=[m_o 0;0 n_o]/2;
        T_p_nor=[m_o;n_o]/2;
        X_t_p=[S_p_nor T_p_nor; 0 0 1]*X_t_nor;
        X_t_p=floor(X_t_p);

end

%基于相机内参的方法，判断目标与视野的位置关系，并将相机坐标系中的坐标转换至图像像素坐标系中
if method==1
    for i=1:1:points_num
        %水平方向的判断
        if X_t_oc(3,i)<=-f_x/c_x*X_t_oc(1,i)&&X_t_oc(1,i)<0
            TARGET_POS_VIEW(2,i)=1;
        elseif X_t_oc(3,i)<=f_x/(m_o-c_x)*X_t_oc(1,i)&&X_t_oc(1,i)>0
            TARGET_POS_VIEW(2,i)=3;
        elseif X_t_oc(3,i)>f_x/(m_o-c_x)*X_t_oc(1,i)&&X_t_oc(3,i)>-f_x/c_x*X_t_oc(1,i)
            TARGET_POS_VIEW(2,i)=2;
        else    
            TARGET_POS_VIEW(2,i)=0;
        end

        %垂直方向的判断
        if X_t_oc(3,i)<=-f_y/c_y*X_t_oc(2,i)&&X_t_oc(2,i)<0
            TARGET_POS_VIEW(3,i)=1;
        elseif X_t_oc(3,i)<=f_y/(n_o-c_y)*X_t_oc(2,i)&&X_t_oc(2,i)>0
            TARGET_POS_VIEW(3,i)=3;
        elseif X_t_oc(3,i)>f_y/(n_o-c_y)*X_t_oc(2,i)&&X_t_oc(3,i)>-f_y/c_y*X_t_oc(2,i)
            TARGET_POS_VIEW(3,i)=2;
        else
            TARGET_POS_VIEW(3,i)=0;
        end
        
        %判断是否有坐标点在视野中
        if TARGET_POS_VIEW(2:3,i)==[2;2]
            TARGET_POS_VIEW(1,i)=2;
            in_view=1;
        end        
    end

        %将相机坐标系中的坐标转换到图像像素坐标系中
        X_t_p=[f_x 0 c_x 0;0 f_y c_y 0;0 0 1 0]*(X_t_oc./X_t_oc(3,:));
        X_t_p=floor(X_t_p);

end

TARGET_POS_VIEW%视野位置关系输出
Alpha_c%最优相机朝向输出
Beta_c%最优相机朝向输出
X_t_p%像素坐标输出

%观察者相机图像绘制
plot(X_t_p(1,:),X_t_p(2,:),'.')
set(gca,'YDir','reverse')
axis equal
axis([0,1920,0,1080]);
text(X_t_p(1,1),X_t_p(2,1),'0','FontSize',8)
text(X_t_p(1,2),X_t_p(2,2),'1','FontSize',8)
text(X_t_p(1,3),X_t_p(2,3),'2','FontSize',8)
text(X_t_p(1,4),X_t_p(2,4),'3','FontSize',8)
text(X_t_p(1,5),X_t_p(2,5),'4','FontSize',8)
text(X_t_p(1,6),X_t_p(2,6),'5','FontSize',8)
text(X_t_p(1,7),X_t_p(2,7),'6','FontSize',8)
text(X_t_p(1,8),X_t_p(2,8),'7','FontSize',8)
text(X_t_p(1,9),X_t_p(2,9),'8','FontSize',8)