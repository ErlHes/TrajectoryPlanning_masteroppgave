settings = struct;
settings.t_sim = 300; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [-400,300,100,400];

settings.t_snapshot = [1,50,100,150,200,250,300];
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange')};