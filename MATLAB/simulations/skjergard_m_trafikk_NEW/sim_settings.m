 settings = struct;
settings.t_sim = 450; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [0,600,0,600];

settings.block = 1;

settings.legend_position = [400,400]';
settings.legend_size = [170,80]';