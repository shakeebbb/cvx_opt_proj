clc
clear all
close all

%% ECE 649 - Convex Optimization Semester Project
% Fall 2018

%% 
Ts = 0.1; % Sampling Time
Th = 10; % Time horizon
N = Th/Ts; % Number of the waypoints
n = 2; % Number of quadrotors
l = 1; % Number of ground robots

% x = [0 0 0 0.0 0 1 0. 0 1]';
% L = L(x)
% eig(L)
% 
P = [-1    -1    0;
    1    0     -1;
    0     1     1];

leader_pose = [0;2;0];  % x,y,z coordinates of the ground robot

cvx_begin sdp
    %variable x(9)
    variable lambda
    variables x(9)
    %variable Lm(3,3) symmetric
    maximize( lambda )
    subject to
        %norm(chol(L(x))*P) >= lambda * eye(2)
        %P' * Lm * P >= lambda * eye(3)
        
        P' * L(x) * P >= lambda * eye(3)
        
%         Lm(1,2) == -min(1,pow_p(e, R - r(1,2,x)));
%         Lm(1,3) == -min(1,pow_p(e, R - r(1,3,x)));
%         Lm(1,1) == -Lm(1,2) - Lm(1,3)
%          
%         Lm(2,2) == -Lm(2,1) - Lm(2,3)
%          
%         Lm(3,3) == -Lm(3,1) - Lm(3,2)
%          
%         Lm(2,3) == 0
        
        x(1:3) == leader_pose
cvx_end
x