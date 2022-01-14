%% Plot situation
home_dir = '/home/emilht/MATLAB/simulator_multiAgent/';
load(strcat(home_dir,'simulations/', simulation, '/sim_output.mat'));
run(strcat(home_dir,'simulations/', simulation,'/sim_settings.m'));
% settings = sim_output.settings;
agent_data = sim_output.agent_data;
time = sim_output.time;
num_agents = size(agent_data,2);
%%
figure(200)
clf(200)
% axis('equal');
axis(settings.axis)

hold on;
grid on;
if(exist('static_obs', 'var'))
[static_obs_handle] = plot_static_obs(static_obs,200);
end
delta_t_plot = round(1/settings.dt);
for k=1:size(agent_data,2)
   plot(agent_data(1,k).wp(2,:), agent_data(1,k).wp(1,:), '--', 'color', get_rgb('light_grey'));
    
end

agent_track_history = nan(num_agents*2,size(1:delta_t_plot:settings.time_steps,2));


handle_holder = [];
for i=1:delta_t_plot:settings.time_steps
        delete(handle_holder);
    count = 1+ (i-1)/delta_t_plot;
    for k=1:size(agent_data,2)
        agent_track_history((1+(k-1)*2):k*2,count) = agent_data(i,k).eta(1:2); 
        handle_ = plot(agent_track_history(k*2,1:count),agent_track_history(1+(k-1)*2,1:count), '--', 'color',get_rgb('shamrock_green'));
        handle_holder = [handle_holder;handle_];
        if(k==1)
            handle_ = plot_os(agent_data(i,k).eta, 'b',2); % Eta
            handle_holder = [handle_holder;handle_];

        else
           handle_ = plot_ts_from_struct(agent_data(i,k),'r',2);
           handle_holder = [handle_holder;handle_];
        end
    end
    title(strcat("Overview at t =  ", num2str(round(i*settings.dt)), " seconds"));
%     disp(agent_data(i,1).eta);
    pause(0.05)
end

