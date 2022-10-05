% 求解二阶微分方程
% x'' = -x'-x;

% 设置x1=x, x2=dx/dt
% 则dx1/dt = dx/dt, dx2/dt = x'' = -dx/dt-x = -x2-x1;

%设X=[x1;x2], dX/dt = [dx1/dt;dx2/dt]= [x2;-x2-x1];

clear all; clc;

fun = @(t,X) [X(2);-X(2)-X(1)];
tSpan = [0,10];
X0 = [0,1];

[tSolve,XSolve] = ode45(fun,tSpan,X0);

plot(tSolve,XSolve);

