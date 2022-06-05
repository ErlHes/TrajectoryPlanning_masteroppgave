 settings = struct;
%  settings.t_sim = 5;
settings.t_sim = 475; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [0,100,0,100];

settings.block = 1;

settings.legend_position = [75,7]';
settings.legend_size = [170,80]';