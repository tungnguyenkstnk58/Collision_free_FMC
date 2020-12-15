%% This file simulates the stock firmware of crazyfile 2.0 based on 
% https://github.com/bitcraze/crazyflie-firmware
close all
clear
clc
%% Sampling time and simulation time
Ts = 5e-2; Tsim = 100; Ns = round(Tsim/Ts);

%% Parameters and initial states of agents
% Number of quadcopter
N = 4;
% Adjacency matrix
A = zeros(N,N);
A(1,2) = 1;         A(2,1) = A(1,2);  
A(1,3) = 1;         A(3,1) = A(1,3);  
A(1,4) = 1;         A(4,1) = A(1,4);  
A(2,3) = 1;         A(3,2) = A(2,3);
A(2,4) = 1;         A(4,2) = A(2,4);
A(3,4) = 1;         A(4,3) = A(3,4);
A(4,1) = 1;         A(1,4) = A(4,1);
% Degree matrix
D = zeros(N,N);
for i = 1:1:N
   for j = 1:1:N
       D(i,i) = D(i,i) + A(i,j);
   end
end
% Laplacian matrix
L = D - A;
% Delta matrix
Delta = ones(N,N);
%Delta(1,1) = 1;  % Only first agent receives reference
% Gravitational acceleration
g = 9.799; %m/s^2
% Agent has same mass
m = 0.032; %kg
% [Position;velocity] %%%%%%%%%%%%%%%% Changing initial state by hand
x01     = [0.5;0.4;0.0;0;0;0];
x02     = [-1.5;0.8;0.0;0;0;0];
x03     = [-0.4;0.4;0.0;0;0;0];
x04     = [1;0.8;0.0;0;0;0];
att01   = zeros(6,1); att01(3,1) = 0.0;
att02   = zeros(6,1); att02(3,1) = 0.0;
att03   = zeros(6,1); att03(3,1) = 0.0;
att04   = zeros(6,1); att04(3,1) = 0.0;

%% Coefficients
% Outter loop Control
gv  = 1.2; % gamma_v
gp  = 0.5; % gamma_p
% Inner loop Control
% PID attitude
roll_kp = 6;
roll_ki = 3;
roll_kd = 0;
roll_int_limit = 20;
pitch_kp = 6;
pitch_ki = 3;
pitch_kd = 0;
pitch_int_limit = 20;
yaw_kp = 6;
yaw_ki = 1;
yaw_kd = 0.35;
yaw_int_limit = 360;
% PID attitude rate
roll_rate_kp = 250;
roll_rate_ki = 500;
roll_rate_kd = 2.5;
roll_rate_int_limit = 33.3;
pitch_rate_kp = 250;
pitch_rate_ki = 500;
pitch_rate_kd = 2.5;
pitch_rate_int_limit = 33.3;
yaw_rate_kp = 120;
yaw_rate_ki = 16.7;
yaw_rate_kd = 0;
yaw_rate_int_limit = 166.7;

%% Reference
%v0 = [0.1;0.1;0.0];
r  = zeros(3,Ns); 
r(:,1) = [0;0;0];

%% States of mQuad
x   = zeros(6 * N,Ns); x(:,1)   = [x01; x02; x03; x04];
att = zeros(6 * N,Ns); att(:,1) = [att01; att02; att03; att04];
u   = zeros(3 * N,Ns);
u_att = zeros(3 * N,Ns);
thrust = zeros(N,Ns);
roll_d = zeros(N,Ns);
pitch_d = zeros(N,Ns);
yaw_d = ones(N,Ns) * att01(3,1);
roll_rate_d = zeros(N,Ns);
pitch_rate_d = zeros(N,Ns);
yaw_rate_d = zeros(N,Ns);
% Rotation formation
rot_matrix = eye(3);
% ref. depending on time
fp1 = [0.0; 0.0; 0];
fp2 = [-0.9; 0.0; 0];
fp3 = [-0.9; 0.8; 0];
fp4 = [0.0; 0.8; 0];
fp_ref  = [fp1; fp2; fp3; fp4];
fv  = [0;0;0];
fa  = [0;0;0];
% Collision-free parameters
col_thrs = 0.5;
safe_thrs = 0.8;
col_muy = 40;
col_lambda = 0.0;
mulR_Sen = zeros(4*N,Ns);
mulR_Sen_dot = zeros(4*N,Ns);
% Obstacle information
ob_position1 = [1.5;0.0;0.3];

%% Simulation 
figure(7)
set(gcf, 'Position',  [800, 180, 500, 500]);
xlim([-2 2]);
ylim([-2 2]);
%annotation('textarrow',[0.78 0.72],[0.3 0.5],'String','Obstacle','Fontsize',12);
tic;
for k = 1:1:Ns-1
    eta_pk = zeros(3*N,1);
    eta_vk = zeros(3*N,1);
    
    %% Obstacle information
   
    %% Reference v and fp fv
    if k > 40
        v0 = [0.0;0.0;0.0];
        r(:,k+1) = r(:,k) + v0 * Ts;
        rot_matrix  = eye(3);
        matrix_init = zeros(3);
        Delta = ones(N,N);
        %Delta(1,1) = 2; Delta(2,2) = 2;
    else
        v0 = [0.0;0.0;0.0];
        r(:,k+1) = [0; 0; 0.3];
        rot_matrix  = zeros(3);
        matrix_init = eye(3);
        Delta = 1*ones(N,N);
    end   
    %% Calcualte eta_p eta_v 
    for i = 1:1:N
        % Obstacle avoidance
        mulR_Sen(1+(i-1)*4:4*i,k) = multi_range(x(1 + (i-1)*6 : 3 + (i-1)*6,k), ob_position);
        if k > 1
            mulR_Sen_dot(1+(i-1)*4:4*i,k) = (mulR_Sen(1+(i-1)*4:4*i,k) - mulR_Sen(1+(i-1)*4:4*i,k-1)) / Ts;
        else
            mulR_Sen_dot(1+(i-1)*4:4*i,k) = 0;
        end
        % fp and fv
        fp = fp_ref(1 + (i-1)*3 : i*3) * step_fcn(k*Ts, 20*Ts, 80*Ts);
        fv = fp_ref(1 + (i-1)*3 : i*3) * step_fcn_dot(k*Ts, 20*Ts, 80*Ts);
        % eta_p and eta_v
        eta_pk(1 + (i-1)*3 : i*3) = x(1 + (i-1)*6 : 3 + (i-1)*6,k) - rot_matrix * fp - matrix_init * x(1 + (i-1)*6 : 3 + (i-1)*6,1);
        eta_vk(1 + (i-1)*3 : i*3) = x(4 + (i-1)*6 : i*6,k)         - fv;
    end
    
% Calculate u
    for i = 1:1:N
        u_tr = input_2nd_order(r(:,k),v0,fa,eta_pk(1+(i-1)*3:i*3),...
                                            eta_vk(1+(i-1)*3:i*3),gp,gv,Delta(i,i));            
        for j = 1:1:N
            u_tr  = u_tr - gp * L(i,j) * eta_pk(1+(j-1)*3:j*3) ...
                    - gv * L(i,j) * eta_vk(1+(j-1)*3:j*3);
        end
        u_x_c = - bump_fcn_dot(mulR_Sen(1+(i-1)*4,k),col_thrs,safe_thrs,col_muy,col_lambda) * mulR_Sen_dot(1+(i-1)*4,k) ...
              + bump_fcn_dot(mulR_Sen(2+(i-1)*4,k),col_thrs,safe_thrs,col_muy,col_lambda) * mulR_Sen_dot(2+(i-1)*4,k);
        u_y_c = - bump_fcn_dot(mulR_Sen(3+(i-1)*4,k),col_thrs,safe_thrs,col_muy,col_lambda) * mulR_Sen_dot(3+(i-1)*4,k) ...
              + bump_fcn_dot(mulR_Sen(4+(i-1)*4,k),col_thrs,safe_thrs,col_muy,col_lambda) * mulR_Sen_dot(4+(i-1)*4,k);
        u(1+(i-1)*3,k) = step_fcn(mulR_Sen(1+(i-1)*4,k),col_thrs,safe_thrs) ...
                        * step_fcn(mulR_Sen(2+(i-1)*4,k),col_thrs,safe_thrs) ...
                        * u_tr(1) + u_x_c;
        u(2+(i-1)*3,k) = step_fcn(mulR_Sen(3+(i-1)*4,k),col_thrs,safe_thrs) ...
                        * step_fcn(mulR_Sen(i*4,k),col_thrs,safe_thrs) ...
                        * u_tr(2) + u_y_c; 
        u(i*3,k) = u_tr(3);    
        %if k < 250
        %   u(1+(i-1)*3:i*3,k) = u_tr(1:3); 
        %end
    end
    
% Calculate tau thrust phi_d theta_d and 2 PID controllers for inner loop
    for i = 1:1:N
        roll = att(1+(i-1)*6,k); 
        pitch = att(2+(i-1)*6,k); 
        yaw = att(3+(i-1)*6,k); 
        roll_rate = att(4+(i-1)*6,k);
        pitch_rate = att(5+(i-1)*6,k);
        yaw_rate = att(6+(i-1)*6,k);
        [thrust(i,k),roll_d(i,k),pitch_d(i,k)] = output_2nd_order(u(1+(i-1)*3:i*3,k),yaw,m,g,pi/9);
        
        if k > 1
           % PID att
           [roll_rate_d(i,k),roll_e_int] = PID_update(roll_e_int_pre,...
               roll_d(i,k),roll,roll_kp,roll_ki,roll_kd,roll_int_limit,Ts);
           [pitch_rate_d(i,k),pitch_e_int] = PID_update(pitch_e_int_pre,...
               pitch_d(i,k),pitch,pitch_kp,pitch_ki,pitch_kd,pitch_int_limit,Ts);
           [yaw_rate_d(i,k),yaw_e_int] = PID_update(yaw_e_int_pre,...
               yaw_d(i,k),yaw,yaw_kp,yaw_ki,yaw_kd,yaw_int_limit,Ts); 
           roll_e_int_pre = roll_e_int;
           pitch_e_int_pre = pitch_e_int;
           yaw_e_int_pre = yaw_e_int;
           % PID att rate
           [cmd_roll,roll_rate_e_int] = PID_update(roll_rate_e_int_pre,...
               roll_rate_d(i,k),roll_rate,roll_rate_kp,roll_rate_ki,roll_rate_kd,roll_rate_int_limit,Ts);
           [cmd_pitch,pitch_rate_e_int] = PID_update(pitch_rate_e_int_pre,...
               pitch_rate_d(i,k),pitch_rate,pitch_rate_kp,pitch_rate_ki,pitch_rate_kd,pitch_rate_int_limit,Ts);
           [cmd_yaw,yaw_rate_e_int] = PID_update(yaw_rate_e_int_pre,...
               yaw_rate_d(i,k),yaw_rate,yaw_rate_kp,yaw_rate_ki,yaw_rate_kd,yaw_int_limit,Ts);         
           roll_rate_e_int_pre = roll_rate_e_int;
           pitch_rate_e_int_pre = pitch_rate_e_int;
           yaw_rate_e_int_pre = yaw_rate_e_int;
        else
            roll_e_int_pre = roll_d(i,k) - roll;
            pitch_e_int_pre = pitch_d(i,k) - pitch;
            yaw_e_int_pre = yaw_d(i,k) - yaw;
            roll_rate_e_int_pre = 0;
            pitch_rate_e_int_pre = 0;
            yaw_rate_e_int_pre = 0;
            cmd_roll = 0;
            cmd_pitch = 0;
            cmd_yaw = 0;
        end
        cmd_roll_sat = saturation(cmd_roll,-32768,32768);
        cmd_pitch_sat = saturation(cmd_pitch,-32768,32768);
        cmd_yaw_sat = saturation(cmd_yaw,-32768,32768);
        u_att(1+(i-1)*3:i*3,k) = 1e-6 * [cmd_roll_sat;cmd_pitch_sat;cmd_yaw_sat];
    end
%% Dynamics of each agent Quadcopter
    for i = 1:1:N
       [~,x_ode] = ode45(@(t,x_ode) crazyflie2(x_ode,thrust(i,k),u_att(1+(i-1)*3:i*3,k)), [(k-1)*Ts, k*Ts], [x(1+(i-1)*6:i*6,k);att(1+(i-1)*6:i*6,k)]); 
       x(1+(i-1)*6:i*6,k+1)     = x_ode(max(size(x_ode)),1:6)';
       att(1+(i-1)*6:i*6,k+1)   = x_ode(max(size(x_ode)),7:12)';
       % Touching land
       if x(3+(i-1)*6,k+1) <= 0
          x(3+(i-1)*6,k+1) = 0;
       end
    end 
    %disp(k);
%Figure
   if (mod(k,5) == 0)
       hold on
       plot(r(1,k),r(2,k),'o','linewidth',1.4,'color', [0 1 0]);
       for i = 1:1:N 
           hold on
           plot(x(1+(i-1)*6,k),x(2+(i-1)*6,k),'x','MarkerSize',12,'color', [1-i/N 1-i/(N+1) 1-i/(N+2)]);
           hold on
           cir = circle(x(1+(i-1)*6,k),x(2+(i-1)*6,k),d1);
       end
       hold on
       plot(ob_position1(1),ob_position1(2),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
       hold on
       plot(ob_position2(1),ob_position2(2),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
       hold on
       plot(ob_position3(1),ob_position3(2),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
       hold on
       plot(ob_position4(1),ob_position4(2),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
       x_bnd = [x(1,k);x(7,k);x(13,k);x(19,k);x(1,k)];
       y_bnd = [x(2,k);x(8,k);x(14,k);x(20,k);x(2,k)];
       z_bnd = [x(3,k);x(9,k);x(15,k);x(21,k);x(3,k)];
       hold on
       bnd = plot(x_bnd,y_bnd,'LineWidth',1.4,'color', [1 0.2 0.2]);
       hold on
       fp_1n = rot_matrix * fp(1:3);
       plot(r(1,k)+fp_1n(1),r(2,k)+fp_1n(2),'d','color', [0 0 1])
       drawnow;
       if k ~= Ns-2
            delete(bnd);
       end
    end 
end
toc;

%% Figure
%close all
for i = 1:1:3
   figure(i)
   set(gcf, 'Position',  [(i-1)*500 + 50, 600, 400, 400]);
   Leg_Fig_Info = 'Ref.   ';
   %plot(1e-3:Ts:(Tsim/Ts)*Ts,r(i,:)+fp_ref(i),'-','linewidth',4);
   %hold on;
   for j = 1:1:N
   plot(Ts:Ts:(Tsim/Ts)*Ts,x(i+(j-1)*6,:)-fp_ref(i+(j-1)*3,:)-r(i,:),'--','linewidth',2);
   hold on;
   Leg_Fig_Info = [Leg_Fig_Info;['Quad. ' num2str(j)]];
   end
   grid;
   Leg_Fig_Info1 = Leg_Fig_Info(2:N+1,:);
   hL = legend(Leg_Fig_Info1);
   %set(hL,'Position', 'NorthEast');
   xlabel('Time (s)');
   set(gca,'fontsize', 12);
   tz = 12;
   if i == 1
       ylim([-1 2]);
       ylabel('${e_{p,x}}$ (m)','Interpreter','latex','Fontsize',14);
       annotation('textarrow',[0.55 0.4],[0.22 0.3],'String','Collision-free','Fontsize',12);
       annotation('textarrow',[0.52 0.62],[0.5 0.45],'String','Collision-free','Fontsize',12);
       rectangle('Position',[0 -0.8 tz 2.0],'Curvature',0.2,'EdgeColor','r')
       axes('position',[.32 .6 .26 .3])
       box on
       for j = 1:1:N
            plot(Ts:Ts:tz,x(1+(j-1)*6,1:tz/Ts)-fp_ref(i+(j-1)*3),'--','linewidth',2);
            hold on;
       end
       grid
       xlim([0 tz]);
       ylim([-1 1.5]);
       %print(gcf,'Cs1_figure/Cs1_epx.png','-dpng','-r400'); 
   elseif i == 2
       ylim([-0.5 1]);
       ylabel('${e_{p,y}}$ (m)','Interpreter','latex','Fontsize',14);
       annotation('textarrow',[0.62 0.55],[0.58 0.53],'String','Collision-free','Fontsize',12);
       annotation('textarrow',[0.55 0.72],[0.2 0.24],'String','Collision-free','Fontsize',12);
       rectangle('Position',[0 -0.45 tz 1.35],'Curvature',0.2,'EdgeColor','r')
       axes('position',[.32 .61 .26 .3])
       box on
       for j = 1:1:N
            plot(Ts:Ts:tz,x(2+(j-1)*6,1:tz/Ts)-fp_ref(i+(j-1)*3),'--','linewidth',2);
            hold on;
       end
       grid
       xlim([0 tz]);
       ylim([-0.5 1]);
       %print(gcf,'Cs1_figure/Cs1_epy.png','-dpng','-r400');
   else
       ylabel('${e_{p,z}}$ (m)','Interpreter','latex','Fontsize',14);
       rect = [.62 .5 .25 .2];
       set(hL,'Position', rect);
       %print(gcf,'Cs1_figure/Cs1_epz.png','-dpng','-r400');
   end
end

%
figure(4)
set(gcf, 'Position',  [0, 50, 600, 900]);
% 
t_st = [Ts,3,6,9,12,15]/Ts;
for j = 1:1:6
subplot(3,2,j);
xlim([-2 2]);
ylim([-2 2]);
for i = 1:1:N 
   hold on
   plot(x(1+(i-1)*6,t_st(j)),x(2+(i-1)*6,t_st(j)),'x','MarkerSize',15,'color', [1-i/N 1-i/(N+1) 1-i/(N+2)]);
   hold on
   plot(ob_record(1+(i-1)*3,t_st(j)),ob_record(2+(i-1)*3,t_st(j)),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
end
grid;
xlabel('${p_{x}}$ (m)','Interpreter','latex','Fontsize',14);
ylabel('${p_{y}}$ (m)','Interpreter','latex','Fontsize',14);
title(['T = ' num2str(t_st(j)*Ts) 's']);
end
%print(gcf,'Cs1_figure/Cs1_snapshot_FMC.png','-dpng','-r400');

%
figure(5)
set(gcf, 'Position',  [0, 50, 900, 900]);
% 
t_st = [18,24,26,28,30,32,34,36,38]/Ts;
for j = 1:1:9
subplot(3,3,j);
xlim([-2 2]);
ylim([-2 2]);
for i = 1:1:N 
   hold on
   plot(x(1+(i-1)*6,t_st(j)),x(2+(i-1)*6,t_st(j)),'x','MarkerSize',15,'color', [1-i/N 1-i/(N+1) 1-i/(N+2)]);
   hold on
   plot(ob_record(1+(i-1)*3,t_st(j)),ob_record(2+(i-1)*3,t_st(j)),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
end
grid;
xlabel('${p_{x}}$ (m)','Interpreter','latex','Fontsize',14);
ylabel('${p_{y}}$ (m)','Interpreter','latex','Fontsize',14);
title(['T = ' num2str(t_st(j)*Ts) 's']);
end
%print(gcf,'Cs1_figure/Cs1_snapshot_ob1.png','-dpng','-r400');

%
figure(6)
set(gcf, 'Position',  [0, 50, 900, 900]);
% 
t_st = [40,42,44,46,48,49,50,52,54]/Ts;
for j = 1:1:9
subplot(3,3,j);
xlim([-2 2]);
ylim([-2 2]);
for i = 1:1:N 
   hold on
   plot(x(1+(i-1)*6,t_st(j)),x(2+(i-1)*6,t_st(j)),'x','MarkerSize',15,'color', [1-i/N 1-i/(N+1) 1-i/(N+2)]);
   hold on
   plot(ob_record(1+(i-1)*3,t_st(j)),ob_record(2+(i-1)*3,t_st(j)),'o','MarkerSize',8,'color', [0.8 0.2 0.5])
end
grid;
xlabel('${p_{x}}$ (m)','Interpreter','latex','Fontsize',14);
ylabel('${p_{y}}$ (m)','Interpreter','latex','Fontsize',14);
title(['T = ' num2str(t_st(j)*Ts) 's']);
end
%print(gcf,'Cs1_figure/Cs1_snapshot_ob2.png','-dpng','-r400');







%% Mathematical model of crazyflie
function [state_dot] = crazyflie2(state,thrust,u_att)

%% Parameter
g = 9.799;
m = 0.032;
Ix = 2.3951e-5;
Iy = 2.3951e-5;
Iz = 3.2347e-5;

%% System
state_dot = zeros(12,1);
phi     = state(7);
theta   = state(8);
psi     = state(9);
dp      = state(10);
dt      = state(11);
dps     = state(12);

cp = cos(phi);
sp = sin(phi);
ct = cos(theta);
st = sin(theta);

% Attitute model
J = [Ix,0,-Ix*st;
     0,Iy*cp^2 + Iz*sp^2,(Iy - Iz)*cp*sp*ct;
     -Ix*st,(Iy - Iz)*cp*sp*ct,Ix*st^2 + Iy*sp^2*ct^2 + Iz*cp^2*ct^2];
 
c11 = 0;
c12 = (Iy - Iz)*(dt*cp*sp + dps*sp^2*ct) + (Iz - Iy)*dps*cp^2*ct - Ix*dps*ct;
c13 = (Iz - Iy)*dps*cp*sp*ct^2;
c21 = (Iz - Iy)*(dt*cp*sp + dps*sp^2*ct) + (Iy - Iz)*dps*cp^2*ct + Ix*dps*ct;
c22 = (Iz - Iy)*dp*cp*sp;
c23 = -Ix*dps*st*ct + Iy*dps*sp^2*st*ct + Iz*dps*cp^2*st*ct;
c31 = (Iy - Iz)*dps*ct^2*sp*cp - Ix*dt*ct;
c32 = (Iz - Iy)*(dt*cp*sp*st + dp*sp^2*ct) + (Iy - Iz)*dp*cp^2*ct + Ix*dps*st*ct - Iy*dps*sp^2*st*ct - Iz*dps*cp^2*st*ct;
c33 = (Iy - Iz)*dp*cp*sp*ct^2 - Iy*dt*sp^2*ct*st - Iz*dt*cp^2*ct*st + Ix*dt*ct*st;
C = [c11,c12,c13;
     c21,c22,c23;
     c31,c32,c33];
 
eta_2dot = inv(J)*(u_att - C*[dp;dt;dps]);

% Position model
A = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi) 0;
     0 0 1];
A = A/m;

tau1 = thrust * cos(phi) * sin(theta);
tau2 = -thrust * sin(phi);
tau3 = thrust * cos(phi) * cos(theta);

u = A * [tau1;tau2;tau3] - [0;0;g];

A_x = [zeros(3,3) eye(3); zeros(3,3) zeros(3,3)];
B_x = [zeros(3,3); eye(3)];

state_dot(1:6) = A_x * state(1:6) + B_x * u;
state_dot(7:9) = state(10:12);
state_dot(10:12) = eta_2dot;
end

%% PID controller
function [output,x_e_int] = PID_update(x_e_init_pre,x_d,x,kp,ki,kd,limit_int,Ts)
x_e = x_d - x;
x_e_dot = (x_d - x)/Ts;
x_e_int = x_e_init_pre + x_e * Ts;
if x_e_int > limit_int
    x_e_int = limit_int;
elseif x_e_int < -limit_int
    x_e_int = -limit_int;
end

output = kp * x_e + kd * x_e_dot + ki * x_e_int;
end

%% Second order integration system
function u_out = input_2nd_order(r_k,v_k,a_k,eta_p,eta_v,gp,gv,delta)
    u_out  = - gp * delta * (eta_p - r_k) ...
             - gv * delta * (eta_v - v_k) + a_k;
end

%% Calculate thrust forece, attitude desired
function [thrust,roll_d,pitch_d] = output_2nd_order(u_out,yaw,m,g,angle_sat)
      % Calculate tau
      tau1 = m * (cos(yaw) * u_out(1) + sin(yaw) * u_out(2));
      tau2 = m * (-sin(yaw) * u_out(1) + cos(yaw) * u_out(2));
      tau3 = m * (u_out(3) + g);  
      % Calculate thrust phi_ref theta_ref
      thrust = sqrt(tau1^2 + tau2^2 + tau3^2);
      roll_d = -asin(tau2/thrust);
      pitch_d = atan(tau1/tau3);
      % Saturation
        if thrust > 2 * m * g
            thrust = 2 * m * g;
        elseif thrust < 0.1 * m * g
            thrust = 0.1 * m * g;
        end
        if roll_d > angle_sat
            roll_d = angle_sat;
        elseif roll_d < - angle_sat
            roll_d = - angle_sat;
        end
        if pitch_d > angle_sat
            pitch_d = angle_sat;
        elseif pitch_d < - angle_sat
            pitch_d = - angle_sat;
        end
end

%% Saturation
function output = saturation(input,min,max)
    if input > max
        output = max;
        disp('sat max'); 
    elseif input < min
        output = min;
        disp('sat min');
    else 
        output = input;
    end
end

%% Bump function dot
function output = bump_fcn_dot(mr, col_thrs, safe_thrs, muy, lambda)
    if mr > col_thrs
       output = lambda * step_fcn_dot(mr, col_thrs, safe_thrs);
    else
        output = 2 * mr + 3 * col_thrs^3/muy + col_thrs;
        output = - output * (col_thrs - mr)^2;
        output = output/(mr + col_thrs^3/muy)^2;
    end
end

%% Smooth step function
function output = step_fcn(mr, low, up)
    if mr <= low
        output = 0;
    elseif mr >= up
        output = 1;
    else
        output = (mr - low)^3/(up -low)^3;
    end
end

%% Smooth step function dot
function output = step_fcn_dot(mr, low ,up)
    if mr <= low || mr >= up
        output = 0;
    else
        output = 3*(mr - low)^2/(up - low)^3;
    end
end

%% Multiranger information
function output = multi_range(cf_pos,ob_pos)
    % error
    epsilon = 0.05;
    size_ob_pos = size(ob_pos);
    num_ob      = size_ob_pos(2);
    dis_x       = 4.0 * ones(num_ob,1);
    dis_y       = 4.0 * ones(num_ob,1);
    mulRange    = 4.0 * ones(4,1);
    for i = 1:1:num_ob
    % folloing y-axis
        if abs(ob_pos(1,i) - cf_pos(1)) < epsilon
            dis_y(i) = ob_pos(2,i) - cf_pos(2);
            if dis_y(i) > 0 && dis_y(i) < 4.0 && dis_y(i) == min(dis_y)
                mulRange(3) = dis_y(i);
            elseif dis_y(i) < 0 && dis_y(i) > -4.0 && abs(dis_y(i)) == abs(min(dis_y))
                mulRange(4) = -dis_y(i);
            end
        end
    % following x-axis
        if abs(ob_pos(2,i) - cf_pos(2)) < epsilon
            dis_x(i) = ob_pos(1,i) - cf_pos(1);
            if dis_x(i) > 0 && dis_x(i) < 4.0 && dis_x(i) == min(dis_x)
                mulRange(1) = dis_x(i);
            elseif dis_x(i) < 0 && dis_x(i) > -4.0 && abs(dis_x(i)) == abs(min(dis_x))
                mulRange(2) = -dis_x(i);
            end
        end
    end
    % Output
    output = mulRange;
end
