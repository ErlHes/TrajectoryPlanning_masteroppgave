settings = struct;
settings.t_sim = 700; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [-597,700,-400,870];

settings.t_snapshot = 50:50:settings.t_sim;
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange')};

settings.text_time_stamps_position = [-480,-190];
settings.delta_text = 16;