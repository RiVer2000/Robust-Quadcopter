function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% FILL IN YOUR CODE HERE

% Variable Initiation
g = params.gravity;
mass = params.mass;
Ixx = params.Ixx;
minF = params.minF;
maxF = params.maxF;

pos = state.pos;
vel = state.vel;
rot = state.rot;
omega = state.omega;

d_pos = des_state.pos;
d_vel = des_state.vel;
d_acc = des_state.acc;

phic_dot = 0;
phic_ddot = 0;

% Tuning parameters
kpz = 100;
kvz = 25;

kpy = 20;
kvy = 4;

kp_phi = 1000;
kv_phi = 10;


u1 = mass*(g + d_acc(2) + kpz*(d_pos(2) - pos(2)) + kvz*(d_vel(2) - vel(2)));

phic = -(1.0/g)*(d_acc(1) + kpy*(d_pos(1) - pos(1)) + kvy*(d_vel(1) - vel(1)));

u2 = Ixx*(phic_ddot + kp_phi*(phic - rot) + kv_phi*(phic_dot - omega));


end

