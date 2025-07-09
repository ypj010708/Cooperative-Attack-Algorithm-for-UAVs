clc
clear all
close all

%%
figure
axis([0 80 0 80 0 25]);  % 扩大空间范围
axis equal
xlabel('x/(m)')
ylabel('y/(m)')
zlabel('z/(m)')
view(3)
grid on

%上色选择
rng(1)
all_colors = rand(8,3);

%%
%。。。。。。。数据记录。。。。。。
bag=zeros(6,500);   %追击无人机1的信息
bag2=zeros(6,500);  %追击无人机2的信息
bag_target=zeros(6,500);  %目标无人机的信息
bag_obs=zeros(3,500);   %障碍物1的信息
bag_obs2=zeros(3,500);   %障碍物2的信息
bag_obs3=zeros(3,500);   %障碍物3的信息

%%
%。。。。。。。追击无人机初始位置。。。。。。。。
begin1=[10,5,8];      % 追击无人机1
begin2=[70,5,12];     % 追击无人机2

%%
%。。。。。。。目标无人机初始位置和参数。。。。。。。。
target_begin=[40,40,15];  % 目标无人机初始位置
target_velocity=4;        % 目标无人机速度
target_maneuver_freq=0.05; % 机动频率
target_maneuver_amp=2;    % 机动幅度

%%
%。。。。。。。追击参数。。。。。
intercept_distance=3;     % 拦截距离
formation_distance=15;    % 编队距离
cooperation_threshold=20; % 协同阈值距离

%%
%。。。。。。。障碍物位置。。。。。。。。
r_obsmove=12;
v_obsmove=1.5;
w=v_obsmove/r_obsmove;
%障碍物初始化 (3D)
obstacle=[40+r_obsmove*cos(pi/2),40+r_obsmove*cos(-pi/6),40+r_obsmove*cos(-pi*5/6); ...
    35+r_obsmove*sin(pi/2),35+r_obsmove*sin(-pi/6),35+r_obsmove*sin(-pi*5/6); ...
    12, 8, 18];  % 障碍物高度

%%
%。。。。。。。是否可视化无人机。。。。。。。
vision_uav=1;
vision_uav2=1;
vision_target=1;

%%
%。。。。。。。协同标志位。。。。。。。。
flag_co=0;  % 协同追击标志
intercept_success=0;  % 拦截成功标志

%%
%。。。。。。。初始化参数。。。。。。。。。。。
v_chase=5;    % 追击无人机速度
delta_t=0.1;  % 时间间隔
t_end=300;    % 最大仿真时间
iters=1;      % 迭代次数初始化

%%
% 追击无人机当前位置
curr1=begin1';
curr_previous1=curr1;
curr2=begin2';
curr_previous2=curr2;

% 目标无人机当前位置
target_curr=target_begin';
target_previous=target_curr;
target_direction=[1,0.5,0.2]';  % 初始飞行方向
target_direction=target_direction/norm(target_direction);

%% 
testR=v_chase*delta_t;
Q_star=8;   % 障碍物影响半径

num_point=30;       % 3D空间球面上的点数
testPoint=zeros(num_point,3);
testOut=zeros(1,num_point);
step_predict=12;    % 预测步长

%%
pos_predict1=zeros(step_predict,3);  % 追击无人机1预测轨迹
pos_predict2=zeros(step_predict,3);  % 追击无人机2预测轨迹

%%
%。。。。。。。画无人机所需的参数。。。。。。。。。
roll_max=8;
pitch_max=8;
yaw_max=8;

%%
U_k1=zeros(3,1);
U_k2=zeros(3,1);

%%
%。。。。。。。。画初始点。。。。。。。。
hold on;
plot3(begin1(1),begin1(2),begin1(3),'*g','MarkerSize',12);
plot3(begin2(1),begin2(2),begin2(3),'*g','MarkerSize',12);
plot3(target_begin(1),target_begin(2),target_begin(3),'*r','MarkerSize',15);

%% 
%。。。。。。。。。MPC初始参数。。。。。。。。。。。
A=[zeros(3),eye(3);
    zeros(3),zeros(3)]*delta_t+eye(6);

B=[0.5*eye(3)*delta_t^2;eye(3)*delta_t];

N=step_predict;

%%
x_k1=[begin1(1);begin1(2);begin1(3);0*ones(3,1)];
x_k2=[begin2(1);begin2(2);begin2(3);0*ones(3,1)];

%%
Q=[2*eye(3),zeros(3);zeros(3),0.1*eye(3)]; % 增加位置权重
F=[3*eye(3),zeros(3);zeros(3),0.1*eye(3)]; % 终端权重
R=0.1*eye(3);   % 输入权重

u_max=6;    % 最大输入加速度
ub=kron(ones(N,1),[u_max;u_max;u_max]);
lb=-ub;

%%
%。。。。。。。主循环。。。。。。。。。。。。
while iters<=t_end/delta_t && ~intercept_success
    %% 目标无人机运动模型
    % 添加随机机动
    if mod(iters,round(1/target_maneuver_freq/delta_t))==0
        % 随机改变飞行方向
        maneuver=[target_maneuver_amp*(rand-0.5); 
                  target_maneuver_amp*(rand-0.5); 
                  target_maneuver_amp*(rand-0.5)];
        target_direction=target_direction+maneuver;
        target_direction=target_direction/norm(target_direction);
    end
    
    % 边界反弹
    if target_curr(1)<=5 || target_curr(1)>=75
        target_direction(1)=-target_direction(1);
    end
    if target_curr(2)<=5 || target_curr(2)>=75
        target_direction(2)=-target_direction(2);
    end
    if target_curr(3)<=3 || target_curr(3)>=22
        target_direction(3)=-target_direction(3);
    end
    
    % 避障机动
    for j=1:size(obstacle,2)
        dist_to_obs=norm(target_curr-obstacle(:,j));
        if dist_to_obs<Q_star*1.5
            avoid_vec=target_curr-obstacle(:,j);
            avoid_vec=avoid_vec/norm(avoid_vec);
            target_direction=0.7*target_direction+0.3*avoid_vec;
            target_direction=target_direction/norm(target_direction);
        end
    end
    
    % 更新目标无人机位置
    target_previous=target_curr;
    target_curr=target_curr+target_velocity*delta_t*target_direction;
    
    %% 协同策略判断
    dist_between_chasers=norm(curr1-curr2);
    dist1_to_target=norm(curr1-target_curr);
    dist2_to_target=norm(curr2-target_curr);
    
    if dist1_to_target<cooperation_threshold || dist2_to_target<cooperation_threshold
        flag_co=1;
    end
    
    %% 拦截成功判断
    if dist1_to_target<intercept_distance || dist2_to_target<intercept_distance
        intercept_success=1;
        fprintf('拦截成功! 时间: %.1f秒\n', iters*delta_t);
    end
    
    %%
    %。。。。。。。。。删除画图。。。。。。。。。。
    if vision_uav
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    if vision_uav2
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    if vision_target
        delete(findobj('FaceAlpha',0.8));
    end
    
    delete(findobj('color',"green"));
    delete(findobj('color',"magenta"));
    delete(findobj('color',"cyan"));
    delete(findobj('FaceColor','red'));
    delete(findobj('color',all_colors(5,:)));

    %%
    %。。。。。。。。障碍物运动。。。。。。。。。。
    obstacle=[40+r_obsmove*cos(pi/2+w*iters*delta_t), ...
        40+r_obsmove*cos(-pi/6+w*iters*delta_t), ...
        40+r_obsmove*cos(-pi*5/6+w*iters*delta_t); ...
        35+r_obsmove*sin(pi/2+w*iters*delta_t), ...
        35+r_obsmove*sin(-pi/6+w*iters*delta_t), ...
        35+r_obsmove*sin(-pi*5/6+w*iters*delta_t); ...
        12+3*sin(w*iters*delta_t), 8+2*cos(w*iters*delta_t), 18+2*sin(2*w*iters*delta_t)];
    
    bag_obs(:,iters)=obstacle(:,1);
    bag_obs2(:,iters)=obstacle(:,2);
    bag_obs3(:,iters)=obstacle(:,3);
    
    %%
    %。。。。。。。。画轨迹。。。。。。。。。
    % 追击无人机轨迹
    plot3([curr_previous1(1),curr1(1)],[curr_previous1(2),curr1(2)],[curr_previous1(3),curr1(3)], ...
        '-','Color',"#0072BD",'linewidth',3)
    curr_previous1=curr1;
    
    plot3([curr_previous2(1),curr2(1)],[curr_previous2(2),curr2(2)],[curr_previous2(3),curr2(3)], ...
        '-','Color',"#D95319",'linewidth',3)
    curr_previous2=curr2;
    
    % 目标无人机轨迹
    plot3([target_previous(1),target_curr(1)],[target_previous(2),target_curr(2)],[target_previous(3),target_curr(3)], ...
        '-','Color',"#EDB120",'linewidth',4)
    
    %% 
    %。。。。。。。。。追击无人机1路径规划。。。。。。。。。。
    % 预测目标位置
    target_predict=target_curr+target_velocity*delta_t*[1:step_predict]'*target_direction';
    
    % 拦截点计算
    if flag_co
        % 协同拦截：无人机1从前方拦截
        intercept_point=target_predict(min(step_predict,6),:)'+formation_distance/2*[1;0;0];
    else
        % 直接追击最近预测点
        intercept_point=target_predict(min(step_predict,4),:)';
    end
    
    curr_temp=curr1;
    for i=1:step_predict
        for j=1:num_point
            phi = acos(1 - 2*j/num_point);
            theta = pi*(1 + sqrt(5))*j;
            
            testPoint(j,:)=[testR*sin(phi)*cos(theta)+curr_temp(1), ...
                testR*sin(phi)*sin(theta)+curr_temp(2), ...
                testR*cos(phi)+curr_temp(3)];
            
            testOut(:,j)=comput_P_intercept(testPoint(j,:)',intercept_point,obstacle,Q_star,curr2,flag_co);
        end
        [~,num]=min(testOut);
        
        curr_temp=testPoint(num,:)';
        pos_predict1(i,:)=curr_temp';
    end

    %% 
    %。。。。。。。。。追击无人机2路径规划。。。。。。。。。。
    if flag_co
        % 协同拦截：无人机2从侧方拦截
        intercept_point2=target_predict(min(step_predict,6),:)'+formation_distance/2*[0;1;0];
    else
        % 直接追击最近预测点
        intercept_point2=target_predict(min(step_predict,4),:)';
    end
    
    curr_temp=curr2;
    for i=1:step_predict
        for j=1:num_point
            phi = acos(1 - 2*j/num_point);
            theta = pi*(1 + sqrt(5))*j;
            
            testPoint(j,:)=[testR*sin(phi)*cos(theta)+curr_temp(1), ...
                testR*sin(phi)*sin(theta)+curr_temp(2), ...
                testR*cos(phi)+curr_temp(3)];
            
            testOut(:,j)=comput_P_intercept(testPoint(j,:)',intercept_point2,obstacle,Q_star,curr1,flag_co);
        end
        [~,num]=min(testOut);
        
        curr_temp=testPoint(num,:)';
        pos_predict2(i,:)=curr_temp';
    end
    
    %%
    %。。。。。。。。。。画图。。。。。。。。。。
    % 画预测轨迹
    plot3(pos_predict1(:,1),pos_predict1(:,2),pos_predict1(:,3), ...
        'Color',"green",'linewidth',2)
    plot3([pos_predict1(1,1),curr1(1)],[pos_predict1(1,2),curr1(2)],[pos_predict1(1,3),curr1(3)], ...
        'Color',"green",'linewidth',2)
    
    plot3(pos_predict2(:,1),pos_predict2(:,2),pos_predict2(:,3), ...
        'Color',"magenta",'linewidth',2)
    plot3([pos_predict2(1,1),curr2(1)],[pos_predict2(1,2),curr2(2)],[pos_predict2(1,3),curr2(3)], ...
        'Color',"magenta",'linewidth',2)
    
    % 画目标预测轨迹
    plot3(target_predict(:,1),target_predict(:,2),target_predict(:,3), ...
        'Color',"cyan",'linewidth',2,'LineStyle','--')
    
    % 画障碍物
    for j=1:size(obstacle,2)
        plot_obstacle_3d(obstacle(1,j),obstacle(2,j),obstacle(3,j),Q_star/2);
    end
    
    % 画拦截距离圆
    plot_intercept_zone(target_curr(1),target_curr(2),target_curr(3),intercept_distance)
    
    %。。。。。。。。。。画无人机。。。。。。。。
    if vision_uav
        roll=U_k1(1,1)/u_max*roll_max; 
        pitch=U_k1(2,1)/u_max*pitch_max;
        yaw=U_k1(3,1)/u_max*yaw_max;
        quadrotor_3d(curr1(1),curr1(2),curr1(3),pitch,roll,yaw,1)
    end
    
    if vision_uav2
        roll=U_k2(1,1)/u_max*roll_max; 
        pitch=U_k2(2,1)/u_max*pitch_max;
        yaw=U_k2(3,1)/u_max*yaw_max;
        quadrotor_3d(curr2(1),curr2(2),curr2(3),pitch,roll,yaw,2)
    end
    
    if vision_target
        % 目标无人机用红色显示
        quadrotor_3d(target_curr(1),target_curr(2),target_curr(3),0,0,0,3)
    end
    
    %% 
    %。。。。。。。MPC控制。。。。。。。。。
    % 无人机1
    x_k_bias=zeros((N+1)*6,1);
    x_k_bias(1:3,1)=curr1;
    for k=1:N
        x_k_bias(6*k+1:6*k+3,1)=pos_predict1(k,:)';
    end
    
    [M,C,U_k1] = MPC(A,B,N,x_k1,x_k_bias,Q,R,F,lb,ub);
    
    x=M*x_k1+C*U_k1;
    x_k1=x(7:12,1);
    bag(:,iters)=x_k1;
    curr1=x_k1(1:3,1);
    
    % 无人机2
    x_k_bias=zeros((N+1)*6,1);
    x_k_bias(1:3,1)=curr2;
    for k=1:N
        x_k_bias(6*k+1:6*k+3,1)=pos_predict2(k,:)';
    end
    
    [M,C,U_k2] = MPC(A,B,N,x_k2,x_k_bias,Q,R,F,lb,ub);
    
    x=M*x_k2+C*U_k2;
    x_k2=x(7:12,1);
    bag2(:,iters)=x_k2;
    curr2=x_k2(1:3,1);
    
    % 记录目标无人机状态
    target_state=[target_curr;target_velocity*target_direction];
    bag_target(:,iters)=target_state;
    
    % 显示信息
    if mod(iters,50)==0
        fprintf('时间: %.1f秒, 距离1: %.2f米, 距离2: %.2f米, 协同状态: %d\n', ...
            iters*delta_t, dist1_to_target, dist2_to_target, flag_co);
    end
    
    %%
    pause(0.02);
    iters=iters+1;
end

if intercept_success
    title('拦截成功!','FontSize',16,'Color','red');
else
    title('拦截超时','FontSize',16,'Color','blue');
end

%% 支持函数

% 拦截势场计算函数
function P = comput_P_intercept(pos, target, obstacles, Q_star, partner_pos, cooperation_flag)
    % 拦截势场计算
    % 目标吸引势场
    dist_to_target = norm(pos - target);
    P_att = 0.5 * dist_to_target^2;
    
    % 障碍物排斥势场
    P_rep = 0;
    for i = 1:size(obstacles, 2)
        obs_pos = obstacles(:, i);
        dist = norm(pos - obs_pos);
        if dist < Q_star
            P_rep = P_rep + 50 * (1/dist - 1/Q_star)^2;
        end
    end
    
    % 协同势场
    P_coop = 0;
    if cooperation_flag
        dist_to_partner = norm(pos - partner_pos);
        if dist_to_partner < 5  % 避免碰撞
            P_coop = 100 * (1/dist_to_partner)^2;
        end
    end
    
    P = P_att + P_rep + P_coop;
end

% 3D四旋翼绘制函数
function quadrotor_3d(x, y, z, pitch, roll, yaw, type)
    L = 1.5; % 臂长
    
    % 旋转矩阵
    R_roll = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    R_pitch = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    R = R_yaw * R_pitch * R_roll;
    
    % 四旋翼几何
    quad_points = L * [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 0.3; 0, 0, -0.3]';
    
    % 应用旋转
    rotated_points = R * quad_points;
    
    % 平移到当前位置
    rotated_points(1, :) = rotated_points(1, :) + x;
    rotated_points(2, :) = rotated_points(2, :) + y;
    rotated_points(3, :) = rotated_points(3, :) + z;
    
    % 根据类型选择颜色
    if type == 1
        color = 'b';  % 蓝色 - 追击无人机1
    elseif type == 2
        color = 'g';  % 绿色 - 追击无人机2
    else
        color = 'r';  % 红色 - 目标无人机
    end
    
    % 绘制四旋翼
    plot3([rotated_points(1, 1), rotated_points(1, 2)], ...
          [rotated_points(2, 1), rotated_points(2, 2)], ...
          [rotated_points(3, 1), rotated_points(3, 2)], [color '-'], 'LineWidth', 3);
    plot3([rotated_points(1, 3), rotated_points(1, 4)], ...
          [rotated_points(2, 3), rotated_points(2, 4)], ...
          [rotated_points(3, 3), rotated_points(3, 4)], [color '-'], 'LineWidth', 3);
    plot3([rotated_points(1, 5), rotated_points(1, 6)], ...
          [rotated_points(2, 5), rotated_points(2, 6)], ...
          [rotated_points(3, 5), rotated_points(3, 6)], [color '-'], 'LineWidth', 2);
    
    % 绘制螺旋桨
    [X, Y, Z] = sphere(8);
    X = 0.4 * X; Y = 0.4 * Y; Z = 0.08 * Z;
    
    for i = 1:4
        if type == 3  % 目标无人机
            surf(X + rotated_points(1, i), Y + rotated_points(2, i), Z + rotated_points(3, i), ...
                 'FaceAlpha', 0.8, 'FaceColor', 'red', 'EdgeColor', 'none');
        else  % 追击无人机
            surf(X + rotated_points(1, i), Y + rotated_points(2, i), Z + rotated_points(3, i), ...
                 'FaceAlpha', 0.7, 'FaceColor', color, 'EdgeColor', 'none');
        end
    end
end

% 3D障碍物绘制函数
function plot_obstacle_3d(x, y, z, r)
    [X, Y, Z] = sphere(12);
    X = r * X + x;
    Y = r * Y + y;
    Z = r * Z + z;
    surf(X, Y, Z, 'FaceColor', 'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

% 拦截区域绘制函数
function plot_intercept_zone(x, y, z, r)
    [X, Y, Z] = sphere(15);
    X = r * X + x;
    Y = r * Y + y;
    Z = r * Z + z;
    surf(X, Y, Z, 'FaceColor', 'yellow', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end