function [S_theta] = CPE_safety_from_pose(alpha_cpa, beta_cpa, param)
% Calculates  a safety-score based on the pose of two vessels in an
% encounter.
% INPUTS:   alpha_cpa is the relative bearing of vessel two from vessel one
%           beta_cpa is the relative bearing of vessel one from vessel two
%           param is a struct of parameters            
alpha_cpa = saturate(alpha_cpa, -param.alpha_cut, param.alpha_cut);
S_theta_alpha = (1-cos(alpha_cpa));

beta_cpa = saturate(beta_cpa, -param.beta_cut, param.beta_cut);
S_theta_beta = (1-cos(beta_cpa));


S_theta_max = param.s_theta_max;
S_theta = S_theta_max*S_theta_alpha*S_theta_beta;

end

