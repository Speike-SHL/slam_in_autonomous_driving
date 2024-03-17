%% ui设置
clear;clc;clf;
close all;

f = figure('Position',[300 300 2000 800]);
t = tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
ax1 = nexttile(1,[3 1]);
ax2 = nexttile;
ax3 = nexttile;
ax4 = nexttile;
global an1_1 an1_2 an1_3 an1_4 an2 an3 an4;
an1_1 = animatedline(ax1,'MaximumNumPoints',700,'LineWidth',2);   %轨迹线
an1_2 = animatedline(ax1,'MaximumNumPoints',2,'LineWidth',2,'Color','r'); %x轴
an1_3 = animatedline(ax1,'MaximumNumPoints',2,'LineWidth',2,'Color','g'); %y轴
an1_4 = animatedline(ax1,'MaximumNumPoints',2,'LineWidth',2,'Color','b'); %z轴
an2 = animatedline(ax2,'Color','r','LineWidth',2);
an3 = animatedline(ax3,'Color','g','LineWidth',2);
an4 = animatedline(ax4,'Color','b','LineWidth',2);

%% 2.3节平面小车圆周运动代码matlab实现
% % 取前左上为世界坐标系，一辆车角速度矢量指向Z方向，速度矢量指向车体坐标系的x轴，
% % 画出该车辆在世界系下的运动
% angular_velocity = 10.0;    %角度制角速度
% linear_velocity = 5.0;      %车辆前进速度
% 
% axis(ax1,[-50,50,-20,80,-50,50]);
% axis(ax2,[0,30,-6,6]);
% axis(ax3,[0,30,-6,6]);
% axis(ax4,[0,30,-6,6]);
% view(ax1,-45,45)
% 
% angular_velocity_rad = deg2rad(angular_velocity);   % 弧度制转角度制
% T_wb = [1 0 0 0;
%         0 1 0 0;
%         0 0 1 0;
%         0 0 0 1];                       %初始位姿
% omega_b = [0;0;angular_velocity_rad];   %角速度矢量
% v_b = [linear_velocity;0;0];            %本体系速度
% dt = 0.05;                              %更新间隔时间
% time = 0;                               %当前时间
% 
% while 1
%     % 更新自身位置
%     v_w = T_wb(1:3,1:3) * v_b;
%     T_wb(1:3,4) = T_wb(1:3,4) + v_w * dt;
%     % 更新自身姿态
%     T_wb(1:3,1:3) = T_wb(1:3,1:3) * Exp_SO3(omega_b*dt);
%     time = time + dt;
%     UpdateUi(T_wb,v_w,time);
%     pause(0.05);
% end

%% 课后习题3
% 物体一方面沿Z轴自转，一方面存在body下x方向的初始线速度，又受到-Z方向的重力加速度影响
angular_velocity = 10;      % 沿Z轴转10deg/s
linear_velocity = 5;        % X轴初始线速度为5m/s
g = -9.8;                   % Z轴重力加速度-9.8m/s^2

axis(ax1,[-30,30,-10,60]);
view(ax1,-45,15)

T_wb = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];           % 初始位姿
v_b = [linear_velocity;0;0];% 本体下初始速度
a_w = [0;0;g];              % 世界系加速度
omega_b = [0;0;deg2rad(angular_velocity)];  % 角速度矢量
dt = 0.05;
time = 0;

while 1
    % 更新位置
    T_wb(1:3,4) = T_wb(1:3,4) + T_wb(1:3,1:3) * v_b * dt + 0.5 * a_w * dt * dt;
    % 更新姿态
    T_wb(1:3,1:3) = T_wb(1:3,1:3) * Exp_SO3(omega_b*dt);
    % 更新本体速度 v_b = v_b + a_b * dt; a_b = R_bw * a_w
    v_b = v_b + (T_wb(1:3,1:3)\a_w) * dt;
    % 计算世界系下速度
    v_w = T_wb(1:3,1:3) * v_b;
    time = time + dt;
    UpdateUi(T_wb,v_w,time);
    pause(0.0005);
end


%% 相关函数
function UpdateUi(T_wb,v_w,time)
    global an1_1 an1_2 an1_3 an1_4 an2 an3 an4;
    % 绘制动画
    % 轨迹
    addpoints(an1_1,T_wb(1,4),T_wb(2,4),T_wb(3,4));    
    % 计算坐标轴
    x_length = 0.1 * (an1_1.Parent.XLim(2)-an1_1.Parent.XLim(1)); % 根据绘图范围决定坐标系长度
    y_length = 0.1 * (an1_1.Parent.YLim(2)-an1_1.Parent.YLim(1));
    z_length = 0.1 * (an1_1.Parent.ZLim(2)-an1_1.Parent.ZLim(1));
    point_o = T_wb(1:3,4);                              
    point_x = T_wb(1:3,1:3) * (T_wb(1:3,1:3)' * point_o + [x_length;0;0]);
    point_y = T_wb(1:3,1:3) * (T_wb(1:3,1:3)' * point_o + [0;y_length;0]);
    point_z = T_wb(1:3,1:3) * (T_wb(1:3,1:3)' * point_o + [0;0;z_length]);
    addpoints(an1_2,[point_o(1),point_x(1)],[point_o(2),point_x(2)],[point_o(3),point_x(3)]);
    addpoints(an1_3,[point_o(1),point_y(1)],[point_o(2),point_y(2)],[point_o(3),point_y(3)]);
    addpoints(an1_4,[point_o(1),point_z(1)],[point_o(2),point_z(2)],[point_o(3),point_z(3)]);
    % 三轴速度
    addpoints(an2,time,v_w(1));
    addpoints(an3,time,v_w(2));
    addpoints(an4,time,v_w(3));
    if time>30
        xlim(an2.Parent,[time-29,time+1]);
        xlim(an3.Parent,[time-29,time+1]);
        xlim(an4.Parent,[time-29,time+1]);
    end
    drawnow limitrate
end

function R = Exp_SO3(phi)
    % 指数映射
    theta = norm(phi);
    n = phi/theta;
    R = cos(theta)*eye(3)+(1-cos(theta))*n*n'+sin(theta)*skew(n);
end

function A = skew(a)
    % 反对称矩阵
    A = [0 -a(3) a(2);
        a(3) 0 -a(1);
        -a(2) a(1) 0];
end