function S = CPE_head_on_score(alpha,beta, params)
%CPE_HEAD_ON_SCORE Summary of this function goes here
%   Detailed explanation goes here

S = 100*(sin(alpha)-1)^2 * (sin(beta)-1)^2 / 16;
end

