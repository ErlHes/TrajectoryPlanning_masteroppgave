settings = struct;
settings.t_sim = 500; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [-348,410,-10,140];


settings.t_snapshot = 1:70:500;%[1,70,100,150,200,250,300];
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange')};

settings.text_time_stamps_position = [125,260];
settings.delta_text = 16;
settings.delta_text_x = 75;

settings.gcf_position = [20 100 1600 350];
settings.axes_handle = [-0.11,-0.0,1.21,1.05];

