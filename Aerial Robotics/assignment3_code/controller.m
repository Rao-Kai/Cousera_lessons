function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
kpx = 180;
kdx = 35;
kpz = 90;
kdz = 20;
kpphi = 100;
kdphi = 1.8;

kpy = kpx;
kdy = kdx;
kppsi = kpphi;
kdpsi = kdphi;
kptheta = kpphi;
kdtheta = kdphi;

% =================== Your code goes here ===================

% 根据11式计算
r1_des_ddot = des_state.acc(1) + kdx*(des_state.vel(1)-state.vel(1)) + kpx*(des_state.pos(1)-state.pos(1));
r2_des_ddot = des_state.acc(2) + kdy*(des_state.vel(2)-state.vel(2)) + kpy*(des_state.pos(2)-state.pos(2));
r3_des_ddot = des_state.acc(3) + kdz*(des_state.vel(3)-state.vel(3)) + kpz*(des_state.pos(3)-state.pos(3));

% 根据14式计算
phi_des = (1/params.gravity)*(r1_des_ddot*sin(des_state.yaw) - r2_des_ddot*cos(des_state.yaw));
theta_des = (1/params.gravity)*(r1_des_ddot*cos(des_state.yaw) + r2_des_ddot*sin(des_state.yaw));


% Thrust F就是u1 (pdf中的13式)
% F = 0;
F = params.mass*(params.gravity + r3_des_ddot);
if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

% Moment M就是u2 (pdf中的10式) p q r是三个角的角速度，用state.omega = [p; q; r]表示
% 正如pdf第5页说到，控制时通常考虑无人机的nominal hover state, 即roll 和 pitch很小, 那么p和q的desir就设定为0
% M = zeros(3,1);
M = [kpphi*(phi_des-state.rot(1)) + kdphi*(0-state.omega(1));
     kptheta*(theta_des-state.rot(2)) + kdtheta*(0-state.omega(2));
     kppsi*(des_state.yaw-state.rot(3)) + kdpsi*(des_state.yawdot-state.omega(3))];

% =================== Your code ends here ===================

end
