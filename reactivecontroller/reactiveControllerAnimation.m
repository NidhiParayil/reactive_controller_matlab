function reactiveControllerAnimation(p,t,X,exportVideo,playbackRate)
% Paddle Animation 
% Input
%   p: Simulation constants
%   t: Simulation time vector
%   X: Simulation state vector
%   exportVideo: Should the video be exported? (True/False)
% Output
%   An animation/File
% By Kevin Green 2021

% FPS for playback and video export
FPS = 6; % If your computer cannot plot in realtime lower this (60).

% For SE3
addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
addpath(fullfile(pwd,'..', 'visualization'))

% Create objects

% puckObj = SphereClass(puck_r);
% baseObj = CubeClass([0.5, actuator_h]);
% outputObj = CubeClass([0.3, actuator_h*2/3]);
% springObj = SpringClass;


agentObj = AgentSphereClass(p.agent_r);


obstacleObj = obSphereClass(p.obstacle_r);



% Create a figure handle
h.figure = figure;
figure(h.figure)
hold on
h.axes = axes(h.figure);

%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)



agentObj.plot
obstacleObj.plot
% Figure properties
view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')


% to dO : uncomment and fix later
h.axes.DataAspectRatioMode = 'manual';
h.axes.DataAspectRatio = [1 1 1];
% % Setup videowriter object
if exportVideo
   v = VideoWriter('reactiveControllerAnimation.mp4');
   v.FrameRate = FPS;
   open(v)
end

% Iterate over state data
tic;

for t_plt = t(1):playbackRate*1.0/FPS:t(end)

    x_state = interp1(t',X',t_plt);
    x_pos = x_state(1);
    y_pos = x_state(4);
    x_ob_pos = x_state(7);
    y_ob_pos = x_state(10);
    % theta_agent = x_state(13);
    % theta_ob = x_state(17);

    % plt(1)=plot([x_pos x_pos+p.agent_r*cos(theta_agent)],[y_pos y_pos+p.agent_r*sin(theta_agent)],'b','LineWidth',4);
    % plt(2)=plot([x_ob_pos x_ob_pos+p.obstacle_r*cos(theta_ob)],[y_ob_pos y_ob_pos+p.obstacle_r*sin(theta_ob)],'r','LineWidth',4);
    % Set axis limits (These will respect the aspect ratio set above)
    h.axes.XLim = [-5, 15];
    h.axes.YLim = [-5, 15];

    % h.axes.ZLim = [-1.0, 1.0];
    
    plot(h.axes,x_ob_pos,y_ob_pos,".r");
    plot(h.axes,x_pos,y_pos,".b");
 

    % Set the puck position
    agentObj.resetFrame
    agentObj.globalMove(SE3([x_pos, y_pos, 0]));

    obstacleObj.resetFrame
    obstacleObj.globalMove(SE3([x_ob_pos, y_ob_pos, 0]));

    % Update data

    agentObj.updatePlotData
    obstacleObj.updatePlotData

   


    if exportVideo %Draw as fast as possible for video export
        drawnow
        frame = getframe(h.figure);
        writeVideo(v,frame);
    else % pause until 1/FPS of a second has passed then draw
        while( toc < 1.0/FPS)
            pause(0.002)
        end
        drawnow
        tic;
    end % if exportvideo
    % delete(plt)
end % t_plt it = ...

end % paddleAnimation
