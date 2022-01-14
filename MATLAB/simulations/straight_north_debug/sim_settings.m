settings = struct;
settings.t_sim = 270; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
% settings.axis = 'equal';%[-60,70,-20,250];
settings.axis = [-130,70,-40,300];


settings.t_snapshot = [1,40,80,120,160,200,250];
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange')};

settings.text_time_stamps_position = [130,-120];
settings.delta_text = 16;