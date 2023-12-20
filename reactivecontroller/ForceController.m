function [ forcex, forcey ] = ForceController(X,p,t)
  AgentReaction_forceX = p.ob_mass*X(9);
  AgentReaction_forceY = p.ob_mass*X(12);
  forcex = p.Kp_f *AgentReaction_forceX + p.Kd_f *AgentReaction_forceX/t + p.Ki_f *AgentReaction_forceX *t;
  forcey = p.Kp_f *AgentReaction_forceY + p.Kd_f *AgentReaction_forceY/t+ p.Ki_f *AgentReaction_forceY*t;
end

