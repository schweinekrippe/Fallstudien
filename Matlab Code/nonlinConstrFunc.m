function [c,ceq] = nonlinConstrFunc(initialGuessZ)

cOP = classOptimParam();    % constant Optimization Prameters
ceq = zeros(cOP.n,1);       % Compute nonlinear equalities at z

testingODE = 0; % select 1 to get an plot of the y and v solution of ode45
if testingODE
    testVal = [];
    testY = cell(1,cOP.n); %zeros(4*(cOP.n+2)+1,2*oCP.n);
    testT = cell(1,cOP.n);
end

h = cOP.tf/cOP.n;
n = cOP.n+1;
initialGuessX = initialGuessZ(1:2*n);
initialGuessU = initialGuessZ(2*n+1:4*n);

% multiple shooting
for i = 0:cOP.n-1   % shooting n-1 times
    [t,y] = ode45(@constrODE,[i*h,(i+1)*h],[initialGuessX(2*i+1:2*i+2);initialGuessU(2*i+1:2*i+2)]);
    % equality constraints (Stetigkeitsbed für die Knoten)
    % X(t_{i+1})-X_{i+1} = 0
    % with X(t_{i+1}) being y(end,1:2) the Runge Kutta approximation
    ceq(2*i+1,1) = y(end,1)-initialGuessX(2*i+3);
    ceq(2*i+2,1) = y(end,2)-initialGuessX(2*i+4);
    if testingODE
        testVal(i+1,:) = [y(end,1) y(end,2)];
        testY{i+1} = y(:,1:2);
        testT{i+1} = t;
    end
end

if testingODE
    %     subplot(2,1,1)
    %     plot(testVal(:,1));
    %     title('ODE Solution position y')
    %     subplot(2,1,2)
    %     plot(testVal(:,2));
    %     title('ODE Solution velocity v')
    % plot of multiple shooted position values y
    close all
    figure
    for i = 1:cOP.n
        plot(testT{i},testY{i}(:,1),'b.')
        hold on
    end
end

cCCP = classCarConstantParam();     % constant Car Parameters

% Compute nonlinear inequalities
% u(1) <= R*((u(2)+...)
c = initialGuessU(1:2:2*n-1)-cCCP.R*(initialGuessU(2:2:2*n)+cCCP.F_A(initialGuessX(2:2:2*n))+cCCP.F_R*ones(n,1)+cCCP.m*cCCP.a_max(initialGuessX(2:2:2*n)));
end