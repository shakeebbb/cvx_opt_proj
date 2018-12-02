clc
clear all
close all

%% ECE 649 - Convex Optimization Semester Project
% Fall 2018

%% 
% Assuming single integrator model with 3 states

nRobots = 3; % Total number of nodes in the network
nStatesPerRobot = 6; % Number of states per robot
nStates = nStatesPerRobot * nRobots; % Total states in a network
nInputs = 9; % Total Inputs to the Network
Ts = 0.1; % Sampling Time
Th = 10; % Time horizon
N = Th/Ts + 1; % Number of the waypoints
n = 2; % Number of quadrotors
l = 1; % Number of ground robots
obs_loc = [1;2;1]; % Static obstacle location
obs_width = 0.25; % Obstacle Width / 2
obs_len = 0.25; % Obstacle Length / 2
M = 10; % Big M


% x = [0 0 0 0.0 0 1 0. 0 1]';
% L = L(x)
% eig(L)

% Double Integrator Continuous Model for One Robot 
A_1 = [zeros(3) eye(3); zeros(3,6)];
B_1 = [zeros(3); eye(3)];

% Double Integrator Discrete Model for One Robot 
sys_d = c2d(ss(A_1,B_1,[],[]), Ts);

% Double Integrator Discrete Model for nRobots
AnCell = repmat({sys_d.A}, 1, nRobots);
BnCell = repmat({sys_d.B}, 1, nRobots);
An = blkdiag(AnCell{:});
Bn = blkdiag(BnCell{:});

% Double Integrator Discrete Model for nRobots for N Waypoints
% ANCell = repmat({An}, 1, N-1);
% BNCell = repmat({Bn}, 1, N-1);
% AN = blkdiag(ANCell{:});
% BN = blkdiag(BNCell{:});

% Mixed Integer Constraints
% Am = [zeros(6,4);
%       1  0 -1 0;
%       0 -1  0 1;
%       0  0  0 0;
%       zeros(3,4);
%       zeros(3,4);
%       zeros(3,4);
%       ]';
Am = [0  1  0  -1;
      1  0  -1 0;
      0  0  0  0;]';
b_2 = obs_loc(1)+obs_len;
b_3 = obs_loc(2)-obs_len;
b_4 = obs_loc(1)-obs_len;
b_1 = obs_loc(2)+obs_len;

bm = [b_1 b_2 -b_3 -b_4]';

AMCell = repmat({Am}, 1, N);

AM = blkdiag(AMCell{:});
BM = repmat(bm, N, 1);

% 
P = [-1    -1    0;
    1    0     -1;
    0     1     1];

% A = zeros(3*(n+l)*(N-1));
% B = eye(3*(n+l)*(N-1));
% 
% [sys_d] = c2d(ss(A,B,[],[]), Ts);

leader_pose = [1 2 0 0 0 0]';  % Leader's State
x_init = [leader_pose' 0.0 0 2 0 0 0 0.0 0 2 0 0 0]';
 x_init(7:12) =  [0.6438 1.1557 0 0 0 0]';
%x_init(7:12) =  [0 1 0 0 0 0]';
%% Solving Convex Optimization Problem with Dynamic Constraints using 'cvx'

cvx_begin
    %variable x(9)
    variable X(nStates,N)
    variable U(nInputs,N-1)
    variable wm(4,N) binary
    variable lambda
    variable ux_max
    variable uy_max
    %variable Lm(3,3) symmetric
    minimize( lambda + 0.3*ux_max + 0.3*uy_max)
    subject to
       % for i = N-1 
%        i = N-3;
%         r(1,2,X((i-1)*nStates+1:i*nStates),nStatesPerRobot) <= lambda(i)
       % end
        
        r(1,2,X(:,N),nStatesPerRobot) <= lambda
    
        %r(1,2,X(end-2*nStates+1:end-nStates),nStatesPerRobot) <= lambda(2)
        %r(1,2,X(end-3*nStates+1:end-2*nStates),nStatesPerRobot) <= lambda(3)
        
        X(:,2:N) == An*X(:,1:N-1) + Bn*U(:,1:N-1) % Dynamic Constraints
        
        %eye(nInputs*(N-1))*U <= 4*ones(nInputs*(N-1),1)
        %eye(nInputs*(N-1))*U >= -4*ones(nInputs*(N-1),1)
        %U(9) >= -5
        
        %X(end-nStates+1 : end-nStates+nStatesPerRobot) == leader_pose % Leader Final Conditions
        U(1,:) == zeros(1,N-1)
        U(2,:) == zeros(1,N-1)
        U(3,:) == zeros(1,N-1)
        U(6,:) == zeros(1,N-1)
        U(9,:) == zeros(1,N-1)
%         
         norm(U(4,:)) <= ux_max
         norm(U(5,:)) <= uy_max

%         U(4,:) <= 10*ones(1,N-1)
%         U(5,:) <= 10*ones(1,N-1)
%         
%         U(4,:) >= -10*ones(1,N-1)
%         U(5,:) >= -10*ones(1,N-1)
        
        
%         U(7,:) <= 30*ones(1,N-1)
%         U(8,:) <= 30*ones(1,N-1)
%         
%         U(4,:) >= -30*ones(1,N-1)
%         U(5,:) >= -30*ones(1,N-1)
%         U(7,:) >= -30*ones(1,N-1)
%         U(8,:) >= -30*ones(1,N-1)

        X(:,1) == x_init
       % X(nStatesPerRobot+1:nStatesPerRobot+3) == x_init(nStatesPerRobot+1:nStatesPerRobot+3)
       % X(2*nStatesPerRobot+1:2*nStatesPerRobot+3) == x_init(2*nStatesPerRobot+1:2*nStatesPerRobot+3)
        % Initial Conditions
%         
%         Am = [1  0 -1 0
%               0 -1  0 1
%               0  0  0 0]'; % For each state TODO : Extend to X vector 
        %bm = [obs_loc(1)+obs_len obs_loc(2)-obs_len obs_loc(1)-obs_len obs_loc(2)+obs_len]';
        
        %%ones(4*N,1)' * wm <= 4-1
        %ones(4*N,1)' * wm >= 0
        %wm <= ones(4*N,1)
        %wm >= zeros(4*N,1)
        %%AM * X + M*wm >= BM
% %         
        Am*X(nStatesPerRobot+1:nStatesPerRobot+3,:) + M*wm >= [bm(1)*ones(1,N); bm(2)*ones(1,N); bm(3)*ones(1,N); bm(4)*ones(1,N)]
        
%         Am(1,:) * X(nStatesPerRobot+1:nStatesPerRobot+3,:) + M*wm(1,:) >= bm(1)*ones(1,N)
%         Am(2,:) * X(nStatesPerRobot+1:nStatesPerRobot+3,:) + M*wm(2,:) >= bm(2)*ones(1,N)
%         Am(3,:) * X(nStatesPerRobot+1:nStatesPerRobot+3,:) + M*wm(3,:) >= bm(3)*ones(1,N)
%         Am(4,:) * X(nStatesPerRobot+1:nStatesPerRobot+3,:) + M*wm(4,:) >= bm(4)*ones(1,N)
        
        ones(1,4) * wm <= 3*ones(1,N)
        %ones(1,4) * wm >= zeros(1,N)
        
        %Am * X(4:6,N-1) + M*wm(:,1) >= bm
        %ones(1,4)*wm(:,1)  <= 3
        
        %Am*X(end-nStates+nStatesPerRobot+1 : end-nStates+2*nStatesPerRobot) + M*wm >= bm;
       %Am*X(end-nStatesPerRobot+1:end-nStatesPerRobot+3) + M*wm >= bm;
        %Am*X(end-2*nStatesPerRobot+1:end-2*nStatesPerRobot+3) + M*wm >= bm;
        %r(1,2,X(end-8:end)) >= 0.5;
cvx_end

%% Solving Non-Convex Optimization Problem with Dynamic Constraints using 'fmincon'


%% Plotting Stuff
close all

t = 0:Ts:Th;
subplot(2,2,1);
plot(t, X(1,:));
hold on
plot(t, X(2,:));
plot(t, X(3,:));
hold off
xlabel('Ground Robot (Leader)');
ylabel('Position (m)');
legend('x','y','z');
grid on

subplot(2,2,2);
plot(t, X(nStatesPerRobot+1,:))
hold on
plot(t, X(nStatesPerRobot+2,:))
plot(t, X(nStatesPerRobot+3,:))
hold off
xlabel('Aerial Robot 1 (Follower)');
ylabel('Position (m)');
legend('x','y','z');
grid on
subplot(2,2,3);
plot(t, X(2*nStatesPerRobot+1,:))
hold on
plot(t, X(2*nStatesPerRobot+2,:))
plot(t, X(2*nStatesPerRobot+3,:))
hold off
xlabel('Aerial Robot 2 (Follower)');
ylabel('Position (m)');
legend('x','y','z');
grid on

title('Leaders Position = [1,2,0]')
%Trajectories plot on Euclidean Coordinates
figure

scatter3(X(1,:), X(2,:), X(3,:));
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
xlim([-2 2]);
ylim([-3 3]);
zlim([0 3]);

grid on; hold on;

scatter3(X(nStatesPerRobot+1,:), X(nStatesPerRobot+2,:), X(nStatesPerRobot+3,:));

scatter3(X(2*nStatesPerRobot+1,:), X(2*nStatesPerRobot+2,:), X(2*nStatesPerRobot+3,:));

legend('Ground Robot (Leader)', 'Aerial Robot 1', 'Aerial Robot 2');

% Plot a Cube for Obstacles

%xc=1; yc=1; zc=1;    % coordinated of the center
%L=10;                 % cube size (length of an edge)
alpha=0.8;           % transparency (max=1=opaque)

Xb = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Yb = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Zb = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

Cb='blue';                  % unicolor

Xb = obs_len*2*(Xb-0.5) + obs_loc(1);
Yb = obs_len*2*(Yb-0.5) + obs_loc(2);
Zb = obs_len*2*(Zb-0.5) + obs_loc(3); 

fill3(Xb,Yb,Zb,Cb,'FaceAlpha',alpha);    % draw cube
title('Leaders Position = [1,2,0]')
