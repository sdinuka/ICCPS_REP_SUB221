clc
clear all
close all

x_init = [3;4]; 
gamma = 0.4;
rho = 0.5;

c1 = 5.0058;
c2 = 5.0058;

delta = 0.06;
delta_time = 1;

%T_sim_1 = 30;

load('Poly_Sys_Time_Params.mat','eta_val','phi_val','tau_val')

phi = phi_val; %unsafe
eta = eta_val; %no input
tau = tau_val; %safe

phi = floor(phi);
eta = floor(eta);
tau = ceil(tau);

syms x1 x2 u
x = [x1; x2];

min_val = -10;
max_val = 10;


h =    -0.1973*x1^4 - 0.42741*x1^3*x2 - 0.17451*x1^2*x2^2 - 0.1079*x1*x2^3 ...
  + 8.335e-07*x2^4 - 3.3808e-06*x1^3 - 3.0606e-06*x1^2*x2 - 1.0894*x1 ...
  *x2^2 - 0.43842*x2^3 + 1.1838*x1^2 + 1.2822*x1*x2 + 2.1238*x2^2 ...
  + 5.7966e-07*x1 + 6.5873e-07*x2 - 0.014414;
h = -h;
h1 = h - c1;
h2 = h1 - c2;


[x1_val, x2_val] = meshgrid(min_val:0.5:max_val, min_val:0.5:max_val);

f1 = x2_val; 
f2 = -x1_val+(1/3)*x1_val.^3 - x2_val;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Safe Operation 1                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g_safe = [0.1; 0.1];
K_safe = [13, -14.5];
b_safe = -15;

u1_safe =  K_safe(1)*x1_val + K_safe(2)*x2_val + b_safe;
u2_safe =  K_safe(1)*x1_val + K_safe(2)*x2_val + b_safe;

f1_safe = x2_val + g_safe(1)*u1_safe; 
f2_safe = -x1_val+(1/3)*x1_val.^3 - x2_val + g_safe(2)*u2_safe;

x_old = x_init;
x_data_safe1 = [];
u_data_safe1 = [];


for ii = 0:delta_time:tau
    x_data_safe1 = [x_data_safe1, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [x2_; -x1_+(1/3)*x1_^3 - x2_];
    %u = -20*sin(ii); %must be bounded
    u_safe1 = K_safe*x_ + b_safe; %must be bounded
    %u_safe1 = safety_controller(x_,c1,c2,-min_val, max_val,gamma,rho)
    %u = 50;
    u_data_safe1 = [u_data_safe1 u_safe1];
    
    x_new = (f + g_safe*u_safe1)*delta + x_old;
    
    x_old = x_new;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            UnSafe Operation                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g_unsafe = [0.1; 0.1];
K_unsafe = [-3, -13];
b_unsafe = -20;
u1_unsafe =  K_unsafe(1)*x1_val + K_unsafe(2)*x2_val + b_unsafe;
u2_unsafe =  K_unsafe(1)*x1_val + K_unsafe(2)*x2_val + b_unsafe;
% u1 = -2000;
% u2 = -2000;
f1_unsafe = x2_val + g_unsafe(1)*u1_unsafe; 
f2_unsafe = -x1_val+(1/3)*x1_val.^3 - x2_val + g_unsafe(2)*u2_unsafe;


x_old = x_data_safe1(:, end);

x_data_unsafe = [];
u_data_unsafe = [];


for ii = tau:delta_time:(tau+phi)
    x_data_unsafe = [x_data_unsafe, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [x2_; -x1_+(1/3)*x1_^3 - x2_];
    %u = -20*sin(ii); %must be bounded
    u_unsafe = K_unsafe*x_ + b_unsafe; %must be bounded
    %u_unsafe = 1;
    u_data_unsafe = [u_data_unsafe u_unsafe];
    
    x_new = (f + g_unsafe*u_unsafe)*delta + x_old;
    
    x_old = x_new;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Restart and Reset                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_old = x_data_unsafe(:, end);

x_data = [];


for ii = (tau+phi):delta_time:(tau+phi + eta)
    x_data = [x_data, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [x2_; -x1_+(1/3)*x1_^3 - x2_];
    %u = -20*sin(ii); %must be bounded
    %u = K_unsafe*x_ + b_unsafe; %must be bounded
    %u = 50;
    %u_data = [u_data u];
    
    x_new = (f)*delta + x_old;
    
    x_old = x_new;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Safe Operation 2                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_old = x_data(:, end);

x_data_safe2 = [];


for ii = (tau+phi+eta):delta_time:(tau+phi + eta  + tau)
    x_data_safe2 = [x_data_safe2, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];
    u_data_safe2 = [];

    f = [x2_; -x1_+(1/3)*x1_^3 - x2_];
    %u = -20*sin(ii); %must be bounded
    u_safe2 = 10*K_safe*x_ + b_safe; %must be bounded
    %u_safe2 = safety_controller(x_,c1,c2,-min_val, max_val,gamma,rho)
    %u = 50;
    u_data_safe2 = [u_data_safe2 u_safe2];
    
    x_new = (f + g_safe*u_safe2)*delta + x_old;
    
    x_old = x_new;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Details of the Initial Set and Unsafe Set                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g_x0 = (x1-3)^2 + (x2-4)^2 - 0.64; %Initial set
g_xu = (x1+2)^2 + (x2+4)^2 - 1; %Unsafe set

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


fig = figure(1);
plot1 = ezplot(h == 0, [min_val, max_val]);
hold on
plot2 = ezplot(h1 == 0, [min_val, max_val]);
plot3 = ezplot(h2 == 0, [min_val, max_val]);
% plot4 = ezplot(g_x0 == 0, [min_val, max_val]);
% plot5 = ezplot(g_xu == 0, [min_val, max_val]);
plot6 = quiver(x1_val, x2_val, f1, f2);
plot7 = quiver(x1_val, x2_val, f1_unsafe, f2_unsafe);
plot(x_data_safe1(1,:),x_data_safe1(2,:),'g', 'LineWidth', 2);
plot(x_data_safe1(1,:),x_data_safe1(2,:),'g', 'LineWidth', 2);
plot(x_data_unsafe(1,:),x_data_unsafe(2,:),'r', 'LineWidth', 2);
plot(x_data_unsafe(1,:),x_data_unsafe(2,:),'r', 'LineWidth', 2);
plot(x_data(1,:),x_data(2,:),'b', 'LineWidth', 2);
plot(x_data(1,:),x_data(2,:),'b', 'LineWidth', 2);
plot(x_data_safe2(1,:),x_data_safe2(2,:),'g', 'LineWidth', 2);
plot(x_data_safe2(1,:),x_data_safe2(2,:),'g', 'LineWidth', 2);
%plot2 = ezplot(Const1_der == 0);
%plot3 = ezplot(Const2_der == 0);
%plot4 = ezplot(Const3_der == 0);
hold off
set(plot1,'Color','black', 'LineWidth', 2,'LineStyle','--')
set(plot2,'Color','#0072BD', 'LineWidth', 2,'LineStyle','--')
set(plot3,'Color','#D95319', 'LineWidth', 2,'LineStyle','--')
% set(plot4,'Color','yello', 'LineWidth', 2)
% set(plot5,'Color','red', 'LineWidth', 2)
set(plot6,'Color','cyan', 'LineWidth', 1)
set(plot7,'Color',[0.6350 0.000 0.6350], 'LineWidth', 1)
%legend('h','Const1 der','Const2 der','Const3 der')
lll=legend('h','h_1','h_2');
lll.FontSize = 18;
xlabel('x_1','FontSize',18,'fontweight','bold')
ylabel('x_2','FontSize',18,'fontweight','bold')
set(gca,'FontSize',15)
xticks([-10:2:10])
title('')
saveas(fig,'Fig-5a.jpg')
time1 = 0:delta_time:tau;
time2 = tau:delta_time:(tau+phi);
time3 = (tau+phi):delta_time:(tau+phi + eta);
time4 = (tau+phi+eta):delta_time:(tau+phi + eta  + tau);

% figure(5)
% plot(time1,x_data_safe1(1,:),'g')
% hold on 
% plot(time2,x_data_unsafe(1,:),'r')
% plot(time3,x_data(1,:),'b')
% plot(time4,x_data_safe2(1,:),'g')
% 
% figure(6)
% plot(time1,x_data_safe1(2,:),'g')
% hold on 
% plot(time2,x_data_unsafe(2,:),'r')
% plot(time3,x_data(2,:),'b')
% plot(time4,x_data_safe2(2,:),'g')



