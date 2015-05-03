function f = costFunc(z)
cOP = classOptimParam();        % constant Optimization Prameters
n = cOP.n;
h = cOP.tf/n;

cCCP = classCarConstantParam(); % constant Car Parameters
x = z(1:2*n);                   % x = [y0;v0;...;yn;vn]
u = z(2*n+1:4*n);               % u = [Mwh0;Fb0;...;Mwhn;Fbn]
% objective function
f = cOP.c1/cCCP.R*sum(u(1:2:2*n-1).*x(2:2:2*n)*h)+(cOP.c1*cCCP.F_R-cOP.c2)*x(2*n-1);

end