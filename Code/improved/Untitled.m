clc
clear all
close all

%%
figure
axis([0 50 0 50 0 20]);  % 扩大z轴范围
axis equal
xlabel('x/(m)')
ylabel('y/(m)')
zlabel('z/(m)')
view(3)
grid on

%上色选择
rng(1)
all_colors = rand(6,3);

%%
%。。。。。。。数据记录。。。。。。
bag=zeros(6,300);   %无人机的信息
bag_obs=zeros(3,300);   %障碍物1的信息 (增加z维度)
bag_obs2=zeros(3,300);   %障碍物2的信息 (增加z维度)
bag_obs3=zeros(3,300);   %障碍物3的信息 (增加z维度)

%%
%。。。。。。。数据记录。。。。。。
bag2=zeros(6,300);

%%
%。。。。。。。初始点-终止点。。。。。。。。
begin=[5,0,3];      % 增加z坐标
over=[25,45,15];    % 增加z坐标

%%
%。。。。。。。初始点-终止点。。。。。。。。
begin2=[40,0,8];    % 增加z坐标

%%
%。。。。。。。虚拟导引点。。。。。
r_gui=6;  %虚拟导引半径guide
theta_gui=-0.75*pi;   %虚拟导引点的夹角
phi_gui=0.1*pi;       %虚拟导引点的仰角 (新增)
v_target=0; %目标运动速度
over_gui=over+r_gui*[cos(theta_gui)*cos(phi_gui), sin(theta_gui)*cos(phi_gui), sin(phi_gui)];  %3D虚拟导引点

%%
%。。。。。。。虚拟导引点。。。。。
theta_gui2=-0.25*pi;   %虚拟导引点的夹角
phi_gui2=-0.15*pi;     %虚拟导引点的仰角 (新增)
over_gui2=over+r_gui*[cos(theta_gui2)*cos(phi_gui2), sin(theta_gui2)*cos(phi_gui2), sin(phi_gui2)];  %3D虚拟导引点

%%
%。。。。。。。障碍物位置。。。。。。。。
r_obsmove=10;
v_obsmove=1;
w=v_obsmove/r_obsmove;
%障碍物初始化 (3D)
obstacle=[25+r_obsmove*cos(pi/2),25+r_obsmove*cos(-pi/6),25+r_obsmove*cos(-pi*5/6); ...
    20+r_obsmove*sin(pi/2),20+r_obsmove*sin(-pi/6),20+r_obsmove*sin(-pi*5/6); ...
    10, 5, 12];  % 增加z坐标

%%
%。。。。。。。是否可视化无人机。。。。。。。
vision_uav=1;
vision_uav2=1;

%%
%。。。。。。。协同标志位。。。。。。。。
flag_co=0;  %cooperate
flag_co2=0;  %cooperate

%%
accu=0;
accu2=0;
accu_time=0;

%%
%。。。。。。。初始化参数。。。。。。。。。。。
v=3;    %每次迭代的速度m/s
delta_t=0.1;    %时间间隔
t_end=200;  %最大仿真时间
iters=1;    %迭代次数初始化

%%
curr=begin'; %当前位置 (3D)
curr_previous=curr;    %上一刻位置

%%
curr2=begin2'; %当前位置 (3D)
curr_previous2=curr2;    %上一刻位置

%% 
testR=v*delta_t;   %测试球面的半径
Q_star=6;   %障碍物涉及的半径

num_point=26;       %3D空间球面上的点数 (增加点数)
testPoint=zeros(num_point,3);   %周围点的坐标数组(x,y,z)
testOut=zeros(1,num_point); %周围点的势能
step_predict=10; %预测步长

%%
pos_predict=zeros(step_predict,3);  %预测域内的位置数组(x,y,z)
pos_predict2=zeros(step_predict,3);  %预测域内的位置数组(x,y,z)

%%
%。。。。。。。画无人机所需的参数。。。。。。。。。
roll_max=5;
pitch_max=5;
yaw_max=5;      % 新增偏航角

%%
U_k=zeros(3,1);
U_k2=zeros(3,1);

%%
%。。。。。。。。画初始点。。。。。。。。
hold on;
plot3(begin(1),begin(2),begin(3),'*b','MarkerSize',10);
plot3(begin2(1),begin2(2),begin2(3),'*b','MarkerSize',10);

%%
%。。。。。。。画目标的球形范围。。。。。。。。
plot_target(over(1),over(2),over(3),r_gui)

%% 
%。。。。。。。。。MPC初始参数。。。。。。。。。。。
A=[zeros(3),eye(3);
    zeros(3),zeros(3)]*delta_t+ ...
    eye(6);   %状态矩阵A初始条件x_k,权重矩阵Q,R及终端误差矩阵F为输入

B=[0.5*eye(3)*delta_t^2;eye(3)*delta_t];    %输入矩阵B

N=step_predict;    %预测长度

%%
x_k=[begin(1);begin(2);begin(3);
    0*ones(3,1)];   %当前状态，3D初始化

%%
x_k2=[begin2(1);begin2(2);begin2(3);
    0*ones(3,1)];   %当前状态，3D初始化

%%
Q=[eye(3),zeros(3);
    zeros(3),zeros(3)]; %权重矩阵Q

F=[eye(3),zeros(3);
    zeros(3),zeros(3)]; %权重矩阵，终端

R=[zeros(3)];   %权重矩阵，输入

u_max=3;    %最大输入加速度
ub=kron(ones(N,1),[u_max;u_max;u_max]); %输入上限
lb=-ub;

%%
%。。。。。。。主循环。。。。。。。。。。。。
while iters<=t_end/delta_t
    %% 
    %。。。。。。。。。。对协同标志位更新。。。。。。。。。。
    if flag_co==1||norm(curr-over_gui')<2*testR
        flag_co=1;  %可以进行近距离靠近
    end
    
    if flag_co2==1||norm(curr2-over_gui2')<2*testR
        flag_co2=1;  %可以进行近距离靠近
    end
    
    if flag_co&&flag_co2
        accu_time=accu_time+1;
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
    
    %删除预测点序列
    delete(findobj('color',"green"));
    delete(findobj('color',"magenta"));
    %删除障碍物
    delete(findobj('FaceColor','red'));
    %删除虚拟导引点
    delete(findobj('color',all_colors(5,:)));

    %%
    %。。。。。。。。。导引点运动。。。。。。。。。。
    over_gui(1,1)=over_gui(1,1)+v_target*delta_t;
    over_gui2(1,1)=over_gui2(1,1)+v_target*delta_t;
    
    %%
    %。。。。。。。。障碍物运动。。。。。。。。。。
    obstacle=[25+r_obsmove*cos(pi/2+w*iters*delta_t), ...
        25+r_obsmove*cos(-pi/6+w*iters*delta_t), ...
        25+r_obsmove*cos(-pi*5/6+w*iters*delta_t); ...
        20+r_obsmove*sin(pi/2+w*iters*delta_t), ...
        20+r_obsmove*sin(-pi/6+w*iters*delta_t), ...
        20+r_obsmove*sin(-pi*5/6+w*iters*delta_t); ...
        10+2*sin(w*iters*delta_t), 5+3*cos(w*iters*delta_t), 12+1.5*sin(2*w*iters*delta_t)];  % 3D运动
    
    bag_obs(:,iters)=obstacle(:,1);
    bag_obs2(:,iters)=obstacle(:,2);
    bag_obs3(:,iters)=obstacle(:,3);
    
    %%
    %。。。。。。。。画轨迹。。。。。。。。。
    plot3([curr_previous(1),curr(1)],[curr_previous(2),curr(2)],[curr_previous(3),curr(3)], ...
        '-','Color',"#4DBEEE",'linewidth',2)
    curr_previous=curr;
    
    plot3([curr_previous2(1),curr2(1)],[curr_previous2(2),curr2(2)],[curr_previous2(3),curr2(3)], ...
        '-','Color',"#D95319",'linewidth',2)
    curr_previous2=curr2;
    
    %。。。。。。。。。。画无人机姿态。。。。。。。。
    if vision_uav
        roll=U_k(1,1)/u_max*roll_max; 
        pitch=U_k(2,1)/u_max*pitch_max;
        yaw=U_k(3,1)/u_max*yaw_max;   % 新增偏航角
        quadrotor(curr(1),curr(2),curr(3),pitch,roll)
    end
    
    if vision_uav2
        roll=U_k2(1,1)/u_max*roll_max; 
        pitch=U_k2(2,1)/u_max*pitch_max;
        yaw=U_k2(3,1)/u_max*yaw_max;   % 新增偏航角
        quadrotor(curr2(1),curr2(2),curr2(3),pitch,roll)
    end

    %% 
    %。。。。。。。。。势场法求解预测点。。。。。。。。。。
    if accu_time<=2
        curr_temp=curr;
        for i=1:step_predict
            %3D球面上的点
            for j=1:num_point
                % 使用球坐标系生成3D点
                phi = acos(1 - 2*j/num_point);  % 极角
                theta = pi*(1 + sqrt(5))*j;     % 方位角 (黄金角)
                
                testPoint(j,:)=[testR*sin(phi)*cos(theta)+curr_temp(1), ...
                    testR*sin(phi)*sin(theta)+curr_temp(2), ...
                    testR*cos(phi)+curr_temp(3)];
                
                testOut(:,j)=comput_P(testPoint(j,:)',over_gui',obstacle, Q_star);
            end
            [~,num]=min(testOut);
            
            curr_temp=testPoint(num,:)';
            pos_predict(i,:)=curr_temp';
        end
    else
        testR=v*delta_t*2/3;
        x_start=over_gui(1,1)-testR*cos(theta_gui)*cos(phi_gui)*accu;
        y_start=over_gui(1,2)-testR*sin(theta_gui)*cos(phi_gui)*accu;
        z_start=over_gui(1,3)-testR*sin(phi_gui)*accu;
        accu=accu+1;
        i=[1:step_predict]';
        
        pos_predict(:,1)=x_start-testR*cos(theta_gui)*cos(phi_gui)*i;
        pos_predict(:,2)=y_start-testR*sin(theta_gui)*cos(phi_gui)*i;
        pos_predict(:,3)=z_start-testR*sin(phi_gui)*i;
    end

    %% 
    %。。。。。。。。。势场法求解预测点2。。。。。。。。。。
    if accu_time<=2
        curr_temp=curr2;
        for i=1:step_predict
            %3D球面上的点
            for j=1:num_point
                phi = acos(1 - 2*j/num_point);
                theta = pi*(1 + sqrt(5))*j;
                
                testPoint(j,:)=[testR*sin(phi)*cos(theta)+curr_temp(1), ...
                    testR*sin(phi)*sin(theta)+curr_temp(2), ...
                    testR*cos(phi)+curr_temp(3)];
                
                testOut(:,j)=comput_P(testPoint(j,:)',over_gui2',obstacle, Q_star);
            end
            [~,num]=min(testOut);
            
            curr_temp=testPoint(num,:)';
            pos_predict2(i,:)=curr_temp';
        end
    else
        x_start=over_gui2(1,1)-testR*cos(theta_gui2)*cos(phi_gui2)*accu2;
        y_start=over_gui2(1,2)-testR*sin(theta_gui2)*cos(phi_gui2)*accu2;
        z_start=over_gui2(1,3)-testR*sin(phi_gui2)*accu2;
        accu2=accu2+1;
        i=[1:step_predict]';
        
        pos_predict2(:,1)=x_start-testR*cos(theta_gui2)*cos(phi_gui2)*i;
        pos_predict2(:,2)=y_start-testR*sin(theta_gui2)*cos(phi_gui2)*i;
        pos_predict2(:,3)=z_start-testR*sin(phi_gui2)*i;
    end
    
    %%
    %。。。。。。。。。。画图。。。。。。。。。。
    %画预测集合
    plot3(pos_predict(:,1),pos_predict(:,2),pos_predict(:,3), ...
        'Color',"green",'linewidth',2)
    plot3([pos_predict(1,1),curr(1)],[pos_predict(1,2),curr(2)],[pos_predict(1,3),curr(3)], ...
        'Color',"green",'linewidth',2)
    
    plot3(pos_predict2(:,1),pos_predict2(:,2),pos_predict2(:,3), ...
        'Color',"magenta",'linewidth',2)
    plot3([pos_predict2(1,1),curr2(1)],[pos_predict2(1,2),curr2(2)],[pos_predict2(1,3),curr2(3)], ...
        'Color',"magenta",'linewidth',2)
    
    %画障碍物
    for j=1:size(obstacle,2)
        plot_obstacle(obstacle(1,j),obstacle(2,j),obstacle(3,j),Q_star/2-0.5);
    end
    
    %画虚拟导引点
    plot3(over_gui(1),over_gui(2),over_gui(3),'*','Color',all_colors(5,:),'MarkerSize',10);
    plot3(over_gui2(1),over_gui2(2),over_gui2(3),'*','Color',all_colors(5,:),'MarkerSize',10);
    
    %% 
    %。。。。。。。MPC求解下一步。。。。。。。。。
    x_k_bias=zeros((N+1)*6,1);
    x_k_bias(1:3,1)=curr;
    for k=1:N
        x_k_bias(6*k+1:6*k+3,1)=pos_predict(k,:)';
    end
    
    [M,C,U_k] = MPC(A,B,N,x_k,x_k_bias,Q,R,F,lb,ub);
    
    x=M*x_k+C*U_k;
    x_k=x(7:12,1);
    bag(:,iters)=x_k;
    curr=x_k(1:3,1);
    
    %% 
    %。。。。。。。MPC求解下一步2。。。。。。。。。
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
    
    %%
    %。。。。。。。。暂停。。。。。。。。。。。。
    pause(0.01);
    iters=iters+1;
    
end
