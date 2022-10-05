% 求解一阶微分方程
% P' = rP(1-P/K)

clear all; clc;

r = 1;
K = 1000;


fun = @(t,P) r*P*(1-P/K); % t是隐函数
tSpan = [0,10];
P0 = 20;

[tSolve,PSolve] = ode45(fun,tSpan,P0);

plot(tSolve,PSolve);