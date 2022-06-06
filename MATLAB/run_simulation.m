
% This scripts runs the miltiAgent simulator

%%
time = 0;
iteration = 1;
bigsim = settings.big;
simplicity = strcat('_Simple_',num2str(settings.simple));
% global tracking_data

disp('Simulation starting... ');
disp('...');
tic;


fig_filename = simulation;
    figure_filepos = strcat('C:\Users\erlen\Documents\GitHub\TrajectoryPlanning_masteroppgave\Rapport\Images\Figures\',simulation,'/');

if(parameters.system.make_video)
    clear('F')
    frame_number = 0;

end

if(visualization)
    figure(600)
    clf(600)

    axis(settings.axis)
    hold on;
    plot_static_obs(static_obs,600);
    title('Simulation')
    xlabel('East [m]');
    ylabel('North [m]');
    grid;

    draw_custom_legend(settings, agents(1,1));
    
    graph_handles = [];
    
end

% if(~exist('extrablock','var'))
%     extrablock = 0;
% end



while time < settings.t_sim
    
%     if time > 10 && (extrablock == 1)
%         run(strcat(home_dir,'simulations/','Blocked_path','/env2.m'));
%         set_global_map_data(static_obs);
%         plot_static_obs(static_obs,600);
%     end
%     if time > 25 && (extrablock == 1)
%         run(strcat(home_dir,'simulations/','Blocked_path','/env.m'));
%         set_global_map_data(static_obs);
%         plot_static_obs(static_obs,600);
%     end
    
    if(visualization) && (vizualization_counter > visualization_interval)
           figure(600)
            hold on;
            delete(graph_handles); 
            graph_handles = [];
            vizualization_counter = 0;  
    end
            
   for j = 1:size(agents,2)
       
        if(agents(j).gnc == 3) || (agents(j).gnc == 4) || (agents(j).gnc == 5)
           tracks = get_tracking_data_full_state(agents(j), agents,parameters.system);
        else
            tracks = get_tracking_data(agents(j), agents);
            
            
            
        end
        
        [agents(j)] = simulate_agent_iteration...
            (agents(j),tracks,settings, parameters, iteration);
        
       
        if(vizualization_counter==0 && visualization)

            if bigsim == 1
                xaxis = [agents(end).eta(2) - 200, agents(end).eta(2) + 200];
                yaxis = [agents(end).eta(1) - 200, agents(end).eta(1) + 200];
                newaxis = [xaxis, yaxis];
                axis(newaxis); % FOR BIG SIMS
            end


            figure(1)
            hold on;
            if bigsim 
                text_handle = text(xaxis(1)+20,yaxis(1)+20,strcat('t =  ', num2str(time),'s'), 'fontsize',22);
            else
                text_handle = text(20,20,strcat('t =  ', num2str(time),'s'), 'fontsize',22);
            end

            graph_handles = [graph_handles;text_handle];
            hold off;
            figure(999)
            hold on;
            if bigsim 
                text_handle = text(xaxis(1)+20,yaxis(1)+20,strcat('t =  ', num2str(time),'s'), 'fontsize',22);
            else
                text_handle = text(20,20,strcat('t =  ', num2str(time),'s'), 'fontsize',22);
            end
            graph_handles = [graph_handles;text_handle];
            hold off;

            if mod(time-0.1,25) < 1
%                 dummy = 'thick';
%             saveas(999,strcat(figure_filepos,simplicity,'fig999_','time=',num2str(time-0.1)),'fig');
%             saveas(1,strcat(figure_filepos,simplicity,'fig1_','time=',num2str(time-0.1)),'fig');
            end

            if(j==size(agents,2))
%                 figure(600);
%                 hold on;
%                 agent_eta = [agents(j).eta(1:2,1);atan2(agents(j).eta_dot(2,1), agents(j).eta_dot(1,1))];
%                 agent_eta_dot = [agents(j).eta_dot];
%                 handle_ = plot_os(agent_eta, 'b', 2); % Eta
%                 graph_handles = [graph_handles;handle_];
%                 handle_ = quiver(agent_eta(2), agent_eta(1), agent_eta_dot(2),agent_eta_dot(1),10,'b','filled');
%                 graph_handles = [graph_handles;handle_];
%                 plot(agents(size(agents,2)).eta(2,1),agents(size(agents,2)).eta(1,1),'b');
%                 plot_trajectory(agents(j),'blue');  
                % Test
%                 figure(1)
%                 plot_trajectory(agents(j),'blue'); % persistent_OS_traj

            else
%                 figure(600);
%                 hold on;
%                 handle_ = plot_os(agents(j).eta, 'r',2);
%                 graph_handles = [graph_handles;handle_];
%                 handle_= quiver(agents(j).eta(2), agents(j).eta(1), agents(j).eta_dot(2),agents(j).eta_dot(1),10,'r','filled');
%                 graph_handles = [graph_handles;handle_];
%                 plot_trajectory(agents(j),'red');
            end
        end
   end
    agent_data(iteration,:) = agents;
    
    time = time + settings.dt;
    iteration = iteration +1;
    
    
    
    if(parameters.system.make_video) && (vizualization_counter==0 && visualization)
        frame_number = frame_number +1;
%         F(frame_number ) = getframe(600);
        F2(frame_number ) = getframe(1);
        F3(frame_number) = getframe(999);
        drawnow
    end
    vizualization_counter = vizualization_counter +1;
    
end

t = toc;
disp('Simulation done!');
disp(strcat("Elapsed time is ",num2str(t)));





%% Make video file from simulation
if(parameters.system.make_video)
    disp('Saving video...')       
    
%     fig_filename = simulation;
    video_filepos = strcat('C:\Users\erlen\Documents\GitHub\TrajectoryPlanning_masteroppgave\MATLAB\videoresults\Newfigs/');

%     % create the video writer with 1 fps
%     writerObj = VideoWriter( strcat(video_filepos, fig_filename,'_newest_fig600.avi'));
%     writerObj.FrameRate = 20;
%     % set the seconds per image
%     % open the video writer
%     open(writerObj);
%     % write the frames to the video
%     for i=1:length(F)
%         % convert the image to a frame
%         frame = F(i) ;    
%         writeVideo(writerObj, frame);
%     end
%     % close the writer object
%     close(writerObj);
    
    writerObj2 = VideoWriter( strcat(video_filepos, fig_filename,simplicity,'fig1.avi'));
    writerObj2.FrameRate = 20;
    
    open(writerObj2);
    for i = 1:length(F2)
        frame = F2(i);
        writeVideo(writerObj2, frame);
    end
    close(writerObj2);
    
        writerObj3 = VideoWriter( strcat(video_filepos, fig_filename,simplicity,'fig999.avi'));
    writerObj3.FrameRate = 20;
    
    open(writerObj3);
    for i = 1:length(F3)
        frame = F3(i);
        writeVideo(writerObj3, frame);
    end
    close(writerObj3);
    
    disp('Video saved.')
    
end

