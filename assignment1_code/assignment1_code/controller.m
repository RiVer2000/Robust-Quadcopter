function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters



% FILL IN YOUR CODE HERE

% Variable Initiation
m = params.mass;
g = params.gravity;
u_max = params.u_max;
u_min = params.u_min;

z_ddot = 0;
E = s_des - s;

%Tuning Parameters
Kp = 195;
Kv = 20;

%Thrust in z Direction
u = m*(z_ddot + Kp*E(1) + Kv*E(2) + g);

%Estabilishing Boundary Conditions
if(u>u_max)
    u = u_max;
end
if(u<u_min)
    u = u_min;
end


end

