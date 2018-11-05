clc
clear all
close all

%% ECE 649 - Convex Optimization Semester Project
% Fall 2018

%% 
% Assuming single integrator model with 3 states

Ts = 0.5; % Sampling Time
Th = 10; % Time horizon
N = Th/Ts + 1; % Number of the waypoints
n = 2; % Number of quadrotors
l = 1; % Number of ground robots

% x = [0 0 0 0.0 0 1 0. 0 1]';
% L = L(x)
% eig(L)

P = [-1    -1    0;
    1    0     -1;
    0     1     1];

leader_pose = [0 2 0]';  % x,y,z coordinates of the ground robot

A = zeros(3*(n+l)*(N-1));
B = eye(3*(n+l)*(N-1));

[sys_d] = c2d(ss(A,B,[],[]), Ts);

x_init = [leader_pose' 0.0 0 1 0.0 0 1]';


%% Solving Convex Optimization Problem with Dynamic Constraints using 'cvx'

cvx_begin sdp
    %variable x(9)
    variable lambda
    variable X(9*(N))
    variable U(9*(N-1))
    %variable Lm(3,3) symmetric
    maximize( lambda )
    subject to
        %norm(chol(L(x))*P) >= lambda * eye(2)
        %P' * Lm * P >= lambda * eye(3)
        
        P' * L(X(end-8:end)) * P >= lambda * eye(3)
        %Y = circshift(X,9);
        %Y(1:9) = x_init;
        
        X(10:end) == sys_d.A*X(1:end-9) + sys_d.B*U
        
%         Lm(1,2) == -min(1,pow_p(e, R - r(1,2,x)));
%         Lm(1,3) == -min(1,pow_p(e, R - r(1,3,x)));
%         Lm(1,1) == -Lm(1,2) - Lm(1,3)
%          
%         Lm(2,2) == -Lm(2,1) - Lm(2,3)
%          
%         Lm(3,3) == -Lm(3,1) - Lm(3,2)Ground Robot
%          
%         Lm(2,3) == 0
        %eye(9*(N-1))*U <= 5*ones(9*(N-1),1)
        %U(9) >= -5
        X(end-8:end-6) == leader_pose
        X(1:9) == x_init
        %r(1,2,X(end-8:end)) <= 0.5;
cvx_end

%% Solving Non-Convex Optimization Problem with Dynamic Constraints using 'fmincon'


%% Plotting Stuff
close all

t = 0:Ts:Th;
subplot(2,2,1);
plot(t, X(1:9:end))
hold on
plot(t, X(2:9:end))
plot(t, X(3:9:end))
hold off
xlabel('Ground Robot (Leader)');
ylabel('Position (m)');
legend('x','y','z');
grid on

subplot(2,2,2);
plot(t, X(4:9:end))
hold on
plot(t, X(5:9:end))
plot(t, X(6:9:end))
hold off
xlabel('Aerial Robot 1 (Follower)');
ylabel('Position (m)');
legend('x','y','z');
grid on

subplot(2,2,3);
plot(t, X(7:9:end))
hold on
plot(t, X(8:9:end))
plot(t, X(9:9:end))
hold off
xlabel('Aerial Robot 2 (Follower)');
ylabel('Position (m)');
legend('x','y','z');
grid on
