clc
clear all
close all

T0 = 10;
U_FR = 49920;
A_FR = 25;
m_F = 6000;
C_PF = 800;
U_RO = 539.61;
A_RO = 48;
m_R = 69.96;
C_PR = 115;

a11 = -(U_FR*A_FR)/(m_F*C_PF);
a12 = -a11;
b11 = 1/(m_F*C_PF);
a21 = (U_FR*A_FR)/(m_R*C_PR);
a22 = -((U_RO*A_RO)/(m_R*C_PR) + a21);
b22 = 1/(m_R*C_PR);
b23 = (U_RO*A_RO)/(m_R*C_PR);

load('WTC_Sys_Time_Params.mat','eta_val','phi_val','tau_val')

phi = phi_val; %unsafe
eta = eta_val; %no input
tau = tau_val; %safe


phi = floor(phi);
eta = floor(eta);
tau = ceil(tau);
c1 = 7;
c2 = 15.5;

c1_orig = 4;
c2_orig = 4;

gamma = 20;
rho = 0.99;
a = 1;

x2_safe_LB = 20;
x2_safe_UB = 30;

delta = 0.06;
delta_time = 1;

x2_safe_LB = 20;
x2_safe_UB = 30;
%h = (x2-x2_safe_LB)*(x2_safe_UB-x2);
r = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)]);
%h = -h;
%h1 = h - c1;
r1 = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)-c1_orig]);
%h2 = h1 - c2;
r2 = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)-c1_orig-c2_orig]);

x_init = [23;25]; 
x_old = x_init;
x_data_safe1 = [];
u_data_safe1 = [];
u_data_safe2 = [];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Safe Operation                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for ii = 0:delta_time:tau
    x_data_safe1 = [x_data_safe1, x_old];
    x1 = x_old(1);
    x2 = x_old(2);
    x = [x1; x2];

    
    h = (x2-x2_safe_LB)*(x2_safe_UB-x2);
%     h2 = (x2-22)*(26-x2);
    h2 = h - c1 - c2;
    dhdx = [0,(x2_safe_UB-x2) - (x2-x2_safe_LB)];


    f = [-1.8087*(x1-x2) ; 0.4628*(x2-T0) + 22.2985*(x1-x2)];
    g = [1/(6000*115) 0; 0 1/(69.96*800)];
    
    u = sdpvar(2,1);
    obj = u'*eye(2)*u;
    F = [];
    F = [F, dhdx*f + dhdx*g*u + a*h >=0, dhdx*f + dhdx*g*u + gamma*sign(h2)*abs(h2)^rho>=0,u(1)>=-10000,u(1)<=10000,u(2)>=-10000,u(2)<=10000];
    ops = sdpsettings('verbose',0);
    optimize(F,obj,ops)
    
    u_safe1 = value(u);
    u_data_safe1 = [u_data_safe1 u_safe1];
    
    x_new = (f + g*u_safe1)*delta + x_old;
    
    x_old = x_new;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            UnSafe Operation                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g_unsafe = [1/(6000*115) 0; 0 1/(69.96*800)];
K_unsafe = [70 100; 100 100];
b_unsafe = 100;



x_old = x_data_safe1(:, end);

x_data_unsafe = [];
u_data_unsafe = [];

kk = 1;
for ii = tau:delta_time:(tau+phi)
    x_data_unsafe = [x_data_unsafe, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [-1.8087*(x1-x2) ; 0.4628*(x2-T0) + 22.2985*(x1-x2)];
    u_unsafe = K_unsafe*x_ + b_unsafe; %must be bounded
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

    f = [-1.8087*(x1-x2) ; 0.4628*(x2-T0) + 22.2985*(x1-x2)];
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


for ii = 0:delta_time:tau
    x_data_safe2 = [x_data_safe2, x_old];
    x1 = x_old(1);
    x2 = x_old(2);
    x = [x1; x2];

    
    h = (x2-x2_safe_LB)*(x2_safe_UB-x2);
    % h2 = (x2-22)*(26-x2);
    h2 = h - c1 - c2;
    dhdx = [0,(x2_safe_UB-x2) - (x2-x2_safe_LB)];


    f = [-1.8087*(x1-x2) ; 0.4628*(x2-T0) + 22.2985*(x1-x2)];
    g = [1/(6000*115) 0; 0 1/(69.96*800)];
    
    u = sdpvar(2,1);
    obj = u'*eye(2)*u;
    F = [];
    F = [F, dhdx*f + dhdx*g*u + a*h >=0, dhdx*f + dhdx*g*u + gamma*sign(h2)*abs(h2)^rho>=0];
    ops = sdpsettings('verbose',0);
    optimize(F,obj,ops)
    
    u_safe2 = value(u);
    u_data_safe2 = [u_data_safe2 u_safe2];
    
    x_new = (f + g*u_safe2)*delta + x_old;
    
    x_old = x_new;

end


%% plots


time1 = 0:delta_time:tau;
time2 = tau:delta_time:(tau+phi);
time3 = (tau+phi):delta_time:(tau+phi + eta);
time4 = (tau+phi+eta):delta_time:(tau+phi + eta  + tau);

fig = figure(6);

p(1)= plot(time1,x_data_safe1(2,:),'g', 'LineWidth', 2);
hold on
p(2)=plot(time2,x_data_unsafe(2,:),'r', 'LineWidth', 2);
p(3)=plot(time3,x_data(2,:),'b', 'LineWidth', 2);
p(4)=plot(time4,x_data_safe2(2,:),'g', 'LineWidth', 2);
p(5) = yline(r(1));
p(6) = yline(r(2));
p(7) = yline(r1(1));
p(8)= yline(r1(2));
p(9)=yline(r2(1));
p(10)= yline(r2(2));
hold off
set(p(5),'Color','black', 'LineWidth', 2,'LineStyle','--')
set(p(6),'Color','black', 'LineWidth', 2,'LineStyle','--')
set(p(7),'Color','#0072BD', 'LineWidth', 2,'LineStyle','--')
set(p(8),'Color','#0072BD', 'LineWidth', 2,'LineStyle','--')
set(p(9),'Color','#D95319', 'LineWidth', 2,'LineStyle','--')
set(p(10),'Color','#D95319', 'LineWidth', 2,'LineStyle','--')
lll = legend(p([5,7,9]),'h','h_1','h_2');
lll.FontSize = 18;
xticks = 0:50:250;
yticks = 19:1:31;
xlim([0 250])
ylim([19 31])
set(gca,'xtick',xticks)
set(gca,'ytick',yticks)
xlabel('Time','FontSize',18)
ylabel('Room Temperature','FontSize',18)
set(gca,'FontSize',15)
saveas(fig,'Fig-4b.jpg')


