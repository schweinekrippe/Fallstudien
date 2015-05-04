function dx = constrODE(~,z)

cCCP = classCarConstantParam();     % constant Car Parameters

x = z(1:2);
u = z(3:4);

dx = zeros(2,1);
% system of differential equations
dx(1) = x(2); 
dx(2) = 1/cCCP.m*(u(1)/cCCP.R-u(2)-cCCP.F_A(x(2)));
% dummy differential equations for u(1),u(2)
dx(3) = 0;
dx(4) = 0;

end