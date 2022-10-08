% qk+1 = qk + akTq˙k;
% q˙k+1 = B^{−1}(qk + akTq˙k)[B(qk)q˙k + f(qk, q˙k)T] + B−1(qk + akTq˙k)Tuk
% q˙k+1 = akq˙k
% x1(k + 1) = x1(k) + akTx2(k)
% x2(k + 1) = B−1(x1(k) + akTx2(k))[B(x1(k))x2(k) + f(x1(k), x2(k))T] + B−1(x1(k) + akTx2(k))Tu(k)
% where x1(k) = qk, x2(k) = ˙qk, and u(k) = uk

clc
clear all
close all

T = 0.01; % sampling time
Ts = 10; % Simulation time
q1 = linspace(0,Ts,(Ts/T)+1);
q2 = linspace(0,Ts,(Ts/T)+1);
q3 = linspace(0,Ts,(Ts/T)+1);
q = [q1', q2', q3']; %joint angles
q_dot = gradient(q)/T; %joint trajectories
q_ref = q_dot; %reference trajectory
for tk=0:Ts/T
    q_ref(tk+1,:) = [(2*pi/3)*sin(tk*T), (pi/2)*sin(0.65*tk*T)+(pi/3), (pi/2)*cos(1.5*tk*T)];
end
plot(0:T:Ts, q_ref)

% system state space model
X = [q q_dot];

%constraints on joints
q1max = (3*pi)/5;
q1min = -pi;
q2max = (3*pi/4);
q2min = -(pi/8);
q3max = pi;
q3min = -(5*pi/12);

%constraints on input
u1max = 800;
u1min = -1000;
u2max = 1400;
u2min = -1600;
u3max = 1000;
u3min = -800;

% using PD controller
Kp = [5000,6000,4000];
Kd = [300,500,100];


function dX = qPDcontroller(X,t,q_ref)
end





