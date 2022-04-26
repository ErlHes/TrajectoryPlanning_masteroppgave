

sys_parameters = struct;

sys_parameters.max_dynamic_obstacles = 20;
sys_parameters.num_dyn_obstacle_attributes = 8; %[ID, N,E,Psi,Speed,Length,Witdh,Time]
% Plotting options
sys_parameters.run_visualization = 0;
sys_parameters.plot_barriers = 0;

sys_parameters.line_of_sight_visibility = false;



% Parameters for making video from simulation
sys_parameters.make_video = true;
