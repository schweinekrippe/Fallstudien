close all; clc

cOP = classOptimParam();    % constant Optimization Prameters
cCCP = classCarConstantParam(); % constant Car Parameters

n = cOP.n;      % number of intervals
tf = cOP.tf;    % time to run from 0 to tf

x = zeros(2*n,1);   % x = [y;v] y: Position v: velocity
u = zeros(2*n,1);   % u = [Mwh;Fb]  Mwh: acceleration   Fb: deceleration
z = [x;u];

[Aineq,bineq,Aeq,beq] = linConstrFunc();

z0 = rand(4*n,1);   % random initial vector
% approximating the initial vector
z0(2) = 0;
z0(2*n) = 0;

% Optimization parameters
options                 = optimoptions('fmincon');
options.Algorithm       = 'sqp';
options.Display         = 'iter';

%z0 = fmincon(@(z) 0, z0,Aineq,bineq,Aeq,beq,[],[],@constrFunc,options);
min = fmincon(@costFunc,z0,Aineq,bineq,Aeq,beq,[],[],@nonlinConstrFunc,options);

figure
subplot(2,1,1)
plot(min(1:2:2*n-1))
title('Solution position y')
subplot(2,1,2)
plot(min(2:2:2*n))
title('Solution velocity v')
