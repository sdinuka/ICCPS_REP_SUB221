%Implementation of Example 2 (SOSTOOLS and its Control Applications - Parrilo, Section 3.4 Safety Verification)

clc
clear all

c1_bar = 10;
c2_bar = 10;

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

u_UB = 1.7e5;
u_LB = 1.7e5;

% Independent variables (x1, x2, u) and decision variables (z1, z2, z3)
% of the set of Eqns. given in the Eqns. (10) in the paper 
syms x1 x2 u1 u2 z1 z2 z3;
% Extract the state varibales into an array x
x = [x1; x2];

% Define the given safe set h(x) here. h(x) is a polynomial in state
% variables x that have lower bounds and upper bounds 
% e.g., consider the safe set defined in the following way 
% -10 <= x1 <= 10
%Then by the definition of CBF: There exists a polynomial function h(x1) s.t.
% h(x1) >= 0 if -10 <= x1 <= 10 (i.e., x1 in safeset)
% h(x2) < 0 if x1 < -10 or x > 10

h = (x2-20)*(30-x2);
%Define the system of the form x_dot = f(x) + g(x)u; f(x) and g(x) symbolically given 
% e.g., f(x) = [x1*x2; x3^2; x3*x1] and  g(x) = [1; 1; 1]
%"u" is independent variable when system is in unsafe region 
%"u = Kx + b" when system is in safe region; "K" and "b" given. e.g., K = [1, 1, 1] and b = 1

f = [a11*x1 + a12*x2 ; a21*x1 + a22*x2];

g = [b11 0 0; 0 b22 b23];
g_input = [b11 0; 0 b22];
g_noise = [0; b23];
K_safe = [7210 0; 0 7210];
b_safe = 1100;

% For Set of Eqns. given in (7)
% program1 = sosprogram([x1], [c1; c2; tau; phi; eta]);


c_UB1 = c1_bar;
c_LB1 = 4;
c_UB2 = c2_bar;
c_LB2 = 4;

epsilon1 = 0.01;
epsilon2 = 0.01;

exit_counter = 0;

while (abs(c_UB1 - c_LB1) >= epsilon1 || abs(c_UB2 - c_LB2) >= epsilon2) && (exit_counter < 30)
    
    
    %flag = 1;
    
    % For Set of Eqns. given in (7)
    program1 = sosprogram([x1, x2, u1, u2], [z1; z2; z3]);
    %[program1, h] = sospolyvar(program1,[x1^2;x1^4],'wscoeff')
    
    c1 = (c_UB1+c_LB1)/2;
    c2 = (c_UB2+c_LB2)/2;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SOSP feasibility
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h1 = h - c1;
    h2 = h1 - c2;

    
    % Set of Eqns. given in (7)
    % Const1 = diff(h2,x1)*(f(1) + K*x + b) - (c1+c2)/tau;
    % Const2 = diff(h,x1)*f(1) + c1/eta;
    % Const3 = diff(h,x1)*(f(1) + u) + c2/phi;

    % Set of Eqns. given in (8)
    
    [program1,l] = sossosvar(program1, [x1; x2; x1^2; x1*x2; x2^2; 1]);
    [program1,p] = sossosvar(program1, [x1; x2; x1^2; x1*x2; x2^2; 1]);
    [program1,q] = sossosvar(program1, [x1; x2; x1^2; x1*x2; x2^2; u1; u2; 1]);
    
    %[program1,l] = sospolyvar(program1,[x1^2; x1*x2; x2^2]);
    %[program1,p] = sospolyvar(program1,[x1^2; x1*x2; x2^2]);
    %[program1,q] = sospolyvar(program1,[x1^2; x2^2; u^2; x1*u; x2*u; x1*x2]);
    
    v = (u_UB-u1)*(u_LB+u1)*(u_UB-u2)*(u_LB+u2); %u is bounded 
    %v = (u_UB-u); %u is only upper bounded 
    
    u_safe = K_safe*x + b_safe;
    Const1 = diff(h2,x1)*(f(1) + g_noise(1)*T0 + g_input(1,1)*u_safe(1)) + diff(h2,x2)*(f(2)+ g_noise(2)*T0 + g_input(2,2)*u_safe(2)) - z3 -l*h; 
    Const2 = diff(h,x1)*f(1) + diff(h,x2)*f(2) + z1 - p*h1;
    Const3 = diff(h,x1)*(f(1) + g_noise(1)*T0 + g_input(1,1)*u1) + diff(h,x2)*(f(2) + g_noise(2)*T0 + g_input(2,2)*u2) + z2 - q*h2*v;
    

    program1 = sosineq(program1, z1);
    program1 = sosineq(program1, z2);
    program1 = sosineq(program1, z3);
    
    program1 = sosineq(program1, 1e-24*Const1);
    program1 = sosineq(program1, 1e-20*Const2);
    program1 = sosineq(program1, 1e-20*Const3);
    
    solver_opt.solver = 'sedumi';
    [program1,info] = sossolve(program1,solver_opt);
    
    if info.pinf == 0 && info.dinf == 0 && info.numerr == 0
        z1_tmp = sosgetsol(program1, z1);
        z2_tmp = sosgetsol(program1, z2);
        z3_tmp = sosgetsol(program1, z3);
    
        eta_val = c1/z1_tmp;
        phi_val = c2/z2_tmp;
        tau_val = (c1 + c2)/z3_tmp;
        
        c_UB1 = c1;
        c_UB2 = c2;
    else
        c_LB1 = c1;
        c_LB2 = c2;
    end
    
    
    exit_counter = exit_counter + 1;
    

    
end

eta_val
phi_val
tau_val

% l_sol = sosgetsol(program1, l)
% p_sol = sosgetsol(program1, p)
% q_sol = sosgetsol(program1, q)
% z1_tmp = sosgetsol(program1, z1)
% z2_tmp = sosgetsol(program1, z2)
% z3_tmp = sosgetsol(program1, z3)


save('WTC_Sys_Time_Params.mat','eta_val','phi_val','tau_val')