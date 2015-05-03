function [c,ceq] = nonlinConstrFunc(initialGuessZ)

ceq = [];       % Compute nonlinear equalities at z
testingODE = 1; % select 1 to get an plot of the y and v solution of ode45
if testingODE
    testVal = [];
end

cOP = classOptimParam();    % constant Optimization Prameters

n = cOP.n;
h = cOP.tf/n;
initialGuessX = initialGuessZ(1:2*n);
initialGuessU = initialGuessZ(2*n+1:4*n);

% multiple shooting
for i = 1:n-1   % shooting n-1 times
    [t,y] = ode45(@constrODE,[i*h,(i+1)*h],[initialGuessX(2*i-1:2*i);initialGuessU(2*i-1:2*i)]);
    % equality constraints (Stetigkeitsbed für die Knoten)
    % X(t_{i+1})-X_{i+1} = 0
    % with X(t_{i+1}) being y(end,1:2) the Runge Kutta approximation
    ceq(2*i-1,1) = y(end,1)-initialGuessX(2*i+1);
    ceq(2*i,1) = y(end,2)-initialGuessX(2*i+2);
    if testingODE
        testVal(i,:) = [y(end,1) y(end,2)];
    end
end

if testingODE
    subplot(2,1,1)
    plot(testVal(:,1));
    title('ODE Solution position y')
    subplot(2,1,2)
    plot(testVal(:,2));
    title('ODE Solution velocity v')
end

cCCP = classCarConstantParam();     % constant Car Parameters

% Compute nonlinear inequalities
% u(1) <= R*((u(2)+...)
c = initialGuessU(1:2:2*n-1)-cCCP.R*(initialGuessU(2:2:2*n)+cCCP.F_A(initialGuessX(2:2:2*n))+cCCP.F_R*ones(n,1)+cCCP.m*cCCP.a_max(initialGuessX(2:2:2*n)));

end