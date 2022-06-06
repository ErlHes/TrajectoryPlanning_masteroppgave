 settings = struct;
settings.t_sim = 200; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [0,200,0, 200];

settings.block = 1;

settings.legend_position = [30,10]';
settings.legend_size = [170,80]';