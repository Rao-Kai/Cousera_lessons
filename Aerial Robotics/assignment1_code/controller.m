function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

kp = 90;
kv = 16;
e = s_des - s;
z_desire_2 = 0; %希望稳定在一个高度，所以二阶导数为0
% u = 0;
u = params.mass*(z_desire_2 + kp*e(1) + kv*e(2) + params.gravity);

% FILL IN YOUR CODE HERE


end

