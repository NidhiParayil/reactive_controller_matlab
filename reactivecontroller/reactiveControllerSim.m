function [t_vec,X_vec,sol_set] = reactiveControllerSim(X0,p,PositionControlFun, ForceControlFun)

% assert( isfield(p,'ma'))
% assert( isfield(p,'e'))
% assert( isfield(p,'F'))


t_start = 0;    % Initial time
t_end = 30;     % Ending time 
dt = 0.01;      % Timestep of the return
% Bind the dynamics function
param_dyn_NC = @(t,X)noncontact_dynamics(t,X,p,PositionControlFun, ForceControlFun);
param_dyn_C = @(t,X)contact_dynamics(t,X,p,PositionControlFun, ForceControlFun);

% Bind the event function
event_fun_contact = @(t,X)contactEvent(t,X,p);
event_fun_noncontact = @(t,X)nonContactEvent(t,X,p);


% Simulation tolerances
options_contact = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun_contact);
options_noncontact = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun_noncontact);

% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};


count =1;
flag=0;    

while t_start < t_end

    if flag ==0
        sol = ode45(param_dyn_NC, [t_start,t_end], X0, options_contact);
    elseif flag ==1
        sol = ode45(param_dyn_C, [t_start,t_end], X0, options_noncontact);
    end
   
    % Apply the hybrid map, calculate the post impact velocity
    if ~isempty(sol.ie) && flag == 0 
        X0 = ContactMap(sol.xe(end),sol.ye(:,end),p);
        flag = 1;
    elseif ~isempty(sol.ie) && flag == 1 
        X0 = nonContactMap(sol.xe(end),sol.ye(:,end),p);
        flag = 0;
    end
    
    sol_set = [sol_set, {sol}];

    time(count)=t_start;
            amp =.5;
    freq=10;
    d= amp.*sin(time(count).*freq);
    vd= amp.*freq.*cos(time(count).*freq);
    fd(count) = -amp.*freq.*freq.*sin(time(count).*freq);
    t_start = sol.x(end);  
    count= count+1;
    
end % simulation while loop
% figure
% 
% subplot(3,1,1)
% plot(time,Force_set_load,'g')
% 
% legend('Force on load')
% 
% subplot(3,1,2)
% plot(time,fd,'r')
% legend('Actuator force')
% 
% subplot(3,1,3)
% plot(time,Force_set_control,'r')
% legend('control')
% xlabel('Time (s)')
% ylabel('Force control')


% plot(time,Force_set_actuator,'g')
% Loop to sample the solution structures and built X_vec
for idx = 1:length(sol_set)
    % This sets up a logical vector so we can perform logical indexing
    t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
    % Evaluate the idx solution structure only at the applicable times
    X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
   % Assign the result to the correct indicies of the return state array
    X_vec(:,t_sample_mask) = X_eval;
       
end

end % paddleSim

function dX = noncontact_dynamics(t,X,p,PositionControlFun, ForceControlFun)

    [clt_x, clt_y] = PositionControlFun(X,t);
      [ control_forceX, control_forceY ]= ForceControlFun(X,t);

    dX = zeros((p.num_obstacles+1)*6,1);
    dX(1) = +X(2);  %agent vel x
    dX(2) = -clt_x; % agent Acc x
    dX(3)=0 ; %jerk x
    dX(4) = +X(5);  %agent vel y
    dX(5) = -clt_y; % agent Acc y
    dX(6)=0 ; %jerk y
    % disp("no contact")


    for i =2:(p.num_obstacles+1)
        dX(6*i-5) = 0; %ob vel x;
        dX(6*i-4) = 0; %ob acc x;
        dX(6*i-3) = 0; %ob jerk x;
        dX(6*i-2) = 0 ;%ob vel y;
        dX(6*i-1) = 0; %ob acc y;
        dX(6*i) = 0; %ob jerk y;

    end

end % dynamics

function dX = contact_dynamics(t,X,p,PositionControlFun, ForceControlFun)


    % [x_intersect, y_intersect]  = get_contact_point(t,X,p);

   [clt_x, clt_y] = PositionControlFun(X,t);
   [ control_forceX, control_forceY ]= ForceControlFun(X,t);

    dX = zeros(12,1);


        dX(2) = control_forceX -dX(8)*p.ob_mass/p.agent_mass-clt_x; % agent Acc x
     
        dX(5) = control_forceY-dX(11)*p.ob_mass/p.agent_mass-clt_y;  %agent acc 
        dX(1) = +X(2);  %agent vel x
        dX(3) = 0;
        dX(4) = X(8);  % agent vel y
        dX(6) = 0;
        dX(7) = X(8);  % ob vel x
        dX(8) = -X(3)*p.agent_mass/p.ob_mass; % ob Acc x
        dX(9) = 0;
        dX(10) = X(11);  % ob acc y
        dX(11) =-X(6)*p.agent_mass/p.ob_mass;  % ob acc y
        dX(12) = 0;
        % disp("pushing")

end % dynamics

function [eventVal,isterminal,direction] = contactEvent(t,X,p)

    distance = sqrt((X(1) - X(7))^2 + (X(4) - X(10))^2);
    val = abs(distance+.1) - (p.agent_r+p.obstacle_r);
    
    eventVal = val;
    isterminal = 1;
    direction =-1;
end % contactEvent


function [eventVal,isterminal,direction] = nonContactEvent(t,X,p)

    distance = sqrt((X(1) - X(7))^2 + (X(4) - X(10))^2);
    val = round((p.agent_r+p.obstacle_r) - distance, 2);
    
    eventVal = val;
    isterminal = 1;
    direction =-1;
end % contactEvent

function X_post = ContactMap(t,X,p)
 
    vob_postX = (2*p.agent_mass/(p.agent_mass+p.ob_mass))*X(2)+((p.ob_mass-p.agent_mass)/(p.agent_mass+p.ob_mass)*X(8));
    vob_postY = (2*p.agent_mass/(p.agent_mass+p.ob_mass))*X(5)+((p.ob_mass-p.agent_mass)/(p.agent_mass+p.ob_mass)*X(11));
    vagent_postX = (2*p.ob_mass/(p.agent_mass+p.ob_mass))*X(8)+((p.agent_mass-p.ob_mass)/(p.agent_mass+p.ob_mass)*X(2));
    vagent_postY = (2*p.ob_mass/(p.agent_mass+p.ob_mass))*X(11)+((p.agent_mass-p.ob_mass)/(p.agent_mass+p.ob_mass)*X(5));
    accAgent_postx =  (vagent_postX -X(2) )/t;
    accAgent_posty =  (vagent_postY -X(5) )/t;
    accOb_postx =  (vob_postX -X(8) )/t;
    accOb_posty =  (vob_postY -X(11) )/t;


    X_post = [X(1), vagent_postX,accAgent_postx, X(4), vagent_postY ,accAgent_posty, X(7), vob_postX ,accOb_postx, X(10), vob_postY, accOb_posty];
end 
function X_post = nonContactMap(t,X,p)
 
  

    X_post = [X(1), X(2), X(3), X(4), X(5),X(6) X(7), X(8),X(9),X(10),X(11),X(12)];
end 




