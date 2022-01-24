%Implementation of Example 2 in the paper 
%System dynamics from the reference: "SOSTOOLS and its Control Applications" by Parrilo, Section 3.4: Safety Verification

clc
clear all

c1_bar = 100;
c2_bar = 100;

u_UB = 36;
u_LB = 135;

% System variables (x1, x2, u) and decision variables (z1, z2, z3)
% of the set of Eqns. given in the Eqns. (10) in the paper 
syms x1 x2 u z1 z2 z3;
% Extract the state varibales into an array x
x = [x1; x2];

% Define the given safe set h(x) here. h(x) is a polynomial in state
% variables x that have lower bounds and upper bounds 
% e.g., consider the safe set defined in the following way 
% -10 <= x1 <= 10
%Then by the definition of CBF: There exists a polynomial function h(x1) s.t.
% h(x1) >= 0 if -10 <= x1 <= 10 (i.e., x1 in safeset)
% h(x2) < 0 if x1 < -10 or x > 10

%%% NOTE: The following control barrier function is generated using the 
%%% MATLAB script "Poly_Sys_Find_CBF.m" in the folder

h = -0.1973*x1^4 - 0.42741*x1^3*x2 - 0.17451*x1^2*x2^2 - 0.1079*x1*x2^3 ...
  + 8.335e-07*x2^4 - 3.3808e-06*x1^3 - 3.0606e-06*x1^2*x2 - 1.0894*x1 ...
  *x2^2 - 0.43842*x2^3 + 1.1838*x1^2 + 1.2822*x1*x2 + 2.1238*x2^2 ...
  + 5.7966e-07*x1 + 6.5873e-07*x2 - 0.014414;
h = -h;

%Define the system of the form x_dot = f(x) + g(x)u; f(x) and g(x) symbolically given 
% e.g., f(x) = [x1*x2; x3^2; x3*x1] and  g(x) = [1; 1; 1]
%"u" is independent variable when system is in unsafe region 
%"u = Kx + b" when system is in safe region; "K" and "b" given. e.g., K = [1, 1, 1] and b = 1

f = [x2; -x1+(1/3)*x1^3 - x2];

g = [0.1; 0.1];
K = [13, -14.5];
b = -15;

% For Set of Eqns. given in (7)
% program1 = sosprogram([x1], [c1; c2; tau; phi; eta]);


c_UB1 = c1_bar;
c_LB1 = 5;
c_UB2 = c2_bar;
c_LB2 = 5;

epsilon1 = 0.01;
epsilon2 = 0.01;

exit_counter = 0;

while (abs(c_UB1 - c_LB1) >= epsilon1 || abs(c_UB2 - c_LB2) >= epsilon2) && (exit_counter < 30)
    
    
    %flag = 1;
    
    % For Set of Eqns. given in (7)
    program1 = sosprogram([x1, x2, u], [z1; z2; z3]);
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
    [program1,q] = sossosvar(program1, [x1; x2; x1^2; x1*x2; x2^2; u; 1]);
    
    %[program1,l] = sospolyvar(program1,[x1^2; x1*x2; x2^2]);
    %[program1,p] = sospolyvar(program1,[x1^2; x1*x2; x2^2]);
    %[program1,q] = sospolyvar(program1,[x1^2; x2^2; u^2; x1*u; x2*u; x1*x2]);
    
    v = (u_UB-u)*(u_LB+u); %u is bounded 
    %v = (u_UB-u); %u is only upper bounded 
    
    u_safe = K*x + b;
    Const1 = diff(h2,x1)*(f(1) + g(1)*u_safe) + diff(h2,x2)*(f(2) + g(2)*u_safe) - z3 -l*h; 
    Const2 = diff(h,x1)*f(1) + diff(h,x2)*f(2) + z1 - p*h1;
    Const3 = diff(h,x1)*(f(1) + g(1)*u) + diff(h,x2)*(f(2) + g(2)*u) + z2 - q*h2*v;
    

    program1 = sosineq(program1, z1);
    program1 = sosineq(program1, z2);
    program1 = sosineq(program1, z3);
    
    program1 = sosineq(program1, 1e-14*Const1);
    program1 = sosineq(program1, 1e-10*Const2);
    program1 = sosineq(program1, 1e-10*Const3);
    
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


save('Poly_Sys_Time_Params.mat','eta_val','phi_val','tau_val')