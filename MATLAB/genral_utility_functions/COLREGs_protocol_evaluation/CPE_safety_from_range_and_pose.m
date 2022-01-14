function S = CPE_safety_from_range_and_pose(range, alpha_cpa, beta_cpa, param)

S_range= CPE_safety_from_range(range,param);
if(range <= param.ranges(1)) % If the dcpa at tcpa is equal to a collision, the geometry at tcpa is irrelevant.
    S_theta = 0;
else
    S_theta = CPE_safety_from_pose(alpha_cpa, beta_cpa, param);
end
S = param.s_range*S_range + param.s_theta*S_theta;

end

