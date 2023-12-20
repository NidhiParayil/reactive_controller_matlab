% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021
clear
clc
close all;
% Initial state conditions
% X0 = [ 0.7 ... % position (m)
%        0.0];   % velocity (m/s)


% Simulation System Constants


p.agent_mass = 1;
p.agent_pos = [0,0];
p.desired_vel = .5;
p.ob_mass = 10;
p.Kp_pos =3;
p.Ki_pos =0.5;
p.Kd_pos =2;
p.Kp_f =1;
p.Ki_f =0;
p.Kd_f =2;
p.agent_r = .5;
p.obstacle_r = 1.5;
p.amplitude = 6;
p.phi = .3;
p.frequency = .4;
p.num_obstacles = 1
% Paddle Trajectory Constants
c.freq = .5; % frequency of paddle oscillation (rad/s)
c.amp = 0.2; % Amplitude of paddle motion (m)

% Bind the trajectory constants to paddle motion function


Xagent=[-1,.0,0,0,0.,0];
Xobts= zeros(1,6*p.num_obstacles);% [-1,.5,0,0.,1,0,2,0,0,0,0,0];
Xobts(1) = 1.5;
Xobts(4)=3;
X0 = [Xagent, Xobts];



% Simulate the system
[t_vec,X_vec,force] = reactiveControllerSim(X0,p,@(X,t)PositionController(X,p,t),@(X,t)ForceController(X,p,t));
% 
%Animate the mass
exportVideo = 1;
playbackRate = 1;
reactiveControllerAnimation(p,t_vec,X_vec,exportVideo,playbackRate);



fig1 = figure;
subplot(3,1,1)

plot(t_vec,X_vec(1,:),'g');
hold on;
plot(t_vec,X_vec(4,:),'b');
plot(t_vec,X_vec(7,:),'r');
plot(t_vec,X_vec(10,:),'m');
title('Position');
legend('agentx', 'agenty','obX','oby')
hold off;

subplot(3,1,2)
plot(t_vec,X_vec(2,:),'g');
hold on;
plot(t_vec,X_vec(5,:),'b');
plot(t_vec,X_vec(8,:),'r');
plot(t_vec,X_vec(11,:),'m');
title('Velocity');
legend('agentx', 'agenty','obX','oby')
hold off;


subplot(3,1,3)
plot(t_vec,X_vec(3,:),'g');
hold on;
plot(t_vec,X_vec(6,:),'b');
plot(t_vec,X_vec(9,:),'r');
plot(t_vec,X_vec(12,:),'m');
title('acceleration');
legend('agentx', 'agenty','obX','oby')
hold off;

f2 =figure;
ax2 = axes(f2);
plot(t_vec,(p.desired_vel *t_vec) -X_vec(1,:),'g');
hold on;
plot(t_vec,(p.desired_vel *t_vec) -X_vec(4,:),'b');
title('position error');
legend('agentx', 'agenty')
hold off;



