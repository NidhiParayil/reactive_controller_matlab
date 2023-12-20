function [ clt_x, clt_y ] = PositionController(X,p, t)


  vel_err_x = -p.Kp_pos * (p.desired_vel *t - X(1)) -p.Kd_pos * (p.desired_vel - X(2)) -p.Ki_pos * (p.desired_vel *t*t/2 - X(1)*t);
  vel_err_y = -p.Kp_pos * (p.desired_vel *t - X(4)) -p.Kd_pos * (p.desired_vel - X(5)) -p.Ki_pos * (p.desired_vel *t*t/2 - X(4)*t);
  clt_x = vel_err_x;
  clt_y = vel_err_y;
end

