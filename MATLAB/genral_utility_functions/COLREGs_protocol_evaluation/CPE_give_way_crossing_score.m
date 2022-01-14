function S = CPE_give_way_crossing_score(alpha, beta)
% Score function that penalizes crossings that are not astern of the vessel
% Score deducted from algorithms 9 and 10.

S = 100*(1-cos(alpha))*(1-sin(beta))/4;

end