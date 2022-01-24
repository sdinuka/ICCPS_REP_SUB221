%Find barriar function of Example 2 (Example from SOSTOOLS and its Control Applications by Parrilo, Section 3.4 Safety Verification)

clc
clear all

pvar x1 x2
vartable = [x1, x2];
program1 = sosprogram(vartable);

f = [x2; -x1+(1/3)*x1^3 - x2];

g_x0 = (x1-3)^2 + (x2-4)^2 - 0.64; %Initial set
g_xu = (x1+2)^2 + (x2+4)^2 - 1; %Unsafe set

[program1,h] = sospolyvar(program1, [x1; x2; x1*x2; x1^2; x2^2; (x1^2)*x2; x1*(x2^2); x1^3; x2^3; (x1^3)*x2; x1*(x2^3); (x1^2)*(x2^2); x1^4; x2^4; 1], 'wscoeff');

[program1,sigma_1] = sossosvar(program1, [x1; x2; x1*x2; x1^2; x2^2; (x1^2)*x2; x1*(x2^2); x1^3; x2^3; (x1^3)*x2; x1*(x2^3); (x1^2)*(x2^2); x1^4; x2^4; 1], 'wscoeff');
[program1,sigma_2] = sossosvar(program1, [x1; x2; x1*x2; x1^2; x2^2; (x1^2)*x2; x1*(x2^2); x1^3; x2^3; (x1^3)*x2; x1*(x2^3); (x1^2)*(x2^2); x1^4; x2^4; 1], 'wscoeff');

Const1 = -h - 0.1 + sigma_1*g_x0;
Const2 = h - 0.1 + sigma_2*g_xu;
Const3 = -(diff(h,x1)*f(1) + diff(h,x2)*f(2));

program1 = sosineq(program1, Const1);
program1 = sosineq(program1, Const2);
program1 = sosineq(program1, Const3);


solver_opt.solver = 'sedumi';
[program1,info] = sossolve(program1,solver_opt);

h_sol = sosgetsol(program1, h)
sigma_1_sol = sosgetsol(program1, sigma_1);
sigma_2_sol = sosgetsol(program1, sigma_2);

