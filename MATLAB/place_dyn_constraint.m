function c_orig = place_dyn_constraint(dynamic_obs,k, i, rad_offset, offsetdist)
    offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + rad_offset;
    offsetdir = [cos(offsetang);sin(offsetang)];
%     offsetdist = 10; % Should ideally be based some function of Involved vessel's speeds
    offsetvektor = offsetdist*offsetdir;
    c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
end