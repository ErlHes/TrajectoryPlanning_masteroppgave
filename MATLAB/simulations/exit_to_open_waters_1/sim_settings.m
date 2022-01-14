settings = struct;
settings.t_sim = 380; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;
settings.axis = [-275,50,260,410];

settings.t_snapshot = 1:364/7:380;%[1,50,100,150,200,250,300];
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange')};


settings.text_time_stamps_position = [300,-270];
settings.delta_text = 10;
settings.delta_text_x = 30;

settings.gcf_position = [20 100 1600 700];
settings.axes_handle = [-0.11,-0.0,1.21,1.05];