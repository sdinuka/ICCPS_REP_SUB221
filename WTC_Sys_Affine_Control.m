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

x_init = [23;25]; 
gamma = 0.4;
rho = 0.5;

c1 = 4;
c2 = 4;

delta = 0.01;
delta_time = 1;

load('WTC_Sys_Time_Params.mat','eta_val','phi_val','tau_val')

phi = phi_val; %unsafe
eta = eta_val; %no input
tau = tau_val; %safe

phi = floor(phi);
eta = floor(eta);
tau = ceil(tau);

syms x1 x2 u
x = [x1; x2];

min_val = -40;
max_val = 40;

x2_safe_LB = 20;
x2_safe_UB = 30;
h = (x2-x2_safe_LB)*(x2_safe_UB-x2);
r = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)]);
%h = -h;
h1 = h - c1;
r1 = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)-c1]);
h2 = h1 - c2;
r2 = roots([-1 (x2_safe_LB+x2_safe_UB) -(x2_safe_LB*x2_safe_UB)-c1-c2]);


[x1_val, x2_val] = meshgrid(min_val:0.5:max_val, min_val:0.5:max_val);

f1 = a11*x1_val + a12*x2_val; 
f2 =  a21*x1_val + a22*x2_val;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            Safe Operation 1                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g_safe = [b11 0; 0 b22];
K_safe = [7210 0; 0 7210];
b_safe = 1100;

% u1_safe =  K_safe(1)*x1_val + K_safe(2)*x2_val + b_safe;
% u2_safe =  K_safe(1)*x1_val + K_safe(2)*x2_val + b_safe;
% 
% f1_safe = x2_val + g_safe(1)*u1_safe; 
% f2_safe = -x1_val+(1/3)*x1_val.^3 - x2_val + g_safe(2)*u2_safe;

x_old = x_init;
x_data_safe1 = [];
u_data_safe1 = [];

p = 6;
Tmin=25;
Tmax=30;
n=length(0:delta_time:tau);
%T = Tmin + (Tmax - Tmin)*sum(rand(n,p),2)/p;
T = T0*ones(1,n);

kk = 1;
for ii = 0:delta_time:tau
    x_data_safe1 = [x_data_safe1, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [a11*x1_ + a12*x2_ ; a21*x1_ + a22*x2_];
    %u = -20*sin(ii); %must be bounded
    u_safe1 = K_safe*x_ + b_safe; %must be bounded
    %u_safe1 = safety_controller(x_,c1,c2,-min_val, max_val,gamma,rho)
    %u = 50;
    u_data_safe1 = [u_data_safe1 u_safe1];
    
    x_new = (f + g_safe*u_safe1 + [0; b23]*T(kk))*delta + x_old;
    
    x_old = x_new;
    kk = kk+1;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            UnSafe Operation                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g_unsafe = [b11 0; 0 b22];
K_unsafe = [700000 100000; 100000 100000];
b_unsafe = 10000;
% u1_unsafe =  K_unsafe(1)*x1_val + K_unsafe(2)*x2_val + b_unsafe;
% u2_unsafe =  K_unsafe(1)*x1_val + K_unsafe(2)*x2_val + b_unsafe;
% % u1 = -2000;
% % u2 = -2000;
% f1_unsafe = x2_val + g_unsafe(1)*u1_unsafe; 
% f2_unsafe = -x1_val+(1/3)*x1_val.^3 - x2_val + g_unsafe(2)*u2_unsafe;


x_old = x_data_safe1(:, end);

x_data_unsafe = [];
u_data_unsafe = [];

kk = 1;
for ii = tau:delta_time:(tau+phi)
    x_data_unsafe = [x_data_unsafe, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];

    f = [a11*x1_ + a12*x2_ ; a21*x1_ + a22*x2_];
    %u = -20*sin(ii); %must be bounded
    u_unsafe = K_unsafe*x_ + b_unsafe; %must be bounded
    %u_unsafe = 0*[1.7e5; -1.7e5];
    %u_unsafe = -1.7e5;
    u_data_unsafe = [u_data_unsafe u_unsafe];
    
    x_new = (f + g_unsafe*u_unsafe+ [0; b23]*T(kk))*delta + x_old;
    
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

    f = [a11*x1_ + a12*x2_ ; a21*x1_ + a22*x2_];
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

kk = 1;
for ii = (tau+phi+eta):delta_time:(tau+phi + eta  + tau)
    x_data_safe2 = [x_data_safe2, x_old];
    x1_ = x_old(1);
    x2_ = x_old(2);
    x_ = [x1_; x2_];
    u_data_safe2 = [];

    f = [a11*x1_ + a12*x2_ ; a21*x1_ + a22*x2_];
    %u = -20*sin(ii); %must be bounded
    u_safe2 = K_safe*x_ + b_safe; %must be bounded
    %u_safe2 = safety_controller(x_,c1,c2,-min_val, max_val,gamma,rho)
    %u = 50;
    u_data_safe2 = [u_data_safe2 u_safe2];
    
    x_new = (f + g_safe*u_safe2 + [0; b23]*T(kk))*delta + x_old;
    
    x_old = x_new;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Details of the Initial Set and Unsafe Set                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g_x0 = (x1-23)^2 + (x2-27)^2 - 0.64; %Initial set
g_xu = (x2-20)*(30-x2)*(x1-20)*(30-x1); %Unsafe set

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




time1 = 0:delta_time:tau;
time2 = tau:delta_time:(tau+phi);
time3 = (tau+phi):delta_time:(tau+phi + eta);
time4 = (tau+phi+eta):delta_time:(tau+phi + eta  + tau);


fig = figure(6);
p(1)= plot(time1,x_data_safe1(2,:),'g','LineWidth',2);
hold on 
p(2)=plot(time2,x_data_unsafe(2,:),'r','LineWidth',2);
p(3)=plot(time3,x_data(2,:),'b','LineWidth',2);
p(4)=plot(time4,x_data_safe2(2,:),'g','LineWidth',2);
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
saveas(fig,'Fig-4a.jpg')

