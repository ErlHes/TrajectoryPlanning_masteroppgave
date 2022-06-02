 settings = struct;
settings.t_sim = 1100; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [0,1400,400,1800];

settings.block = 1;

settings.legend_position = [1500,1000]';
settings.legend_size = [170,80]';