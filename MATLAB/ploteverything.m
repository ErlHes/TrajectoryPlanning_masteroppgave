function plots = ploteverything(loopdata,w_opt, vessel, tracks, reference_trajectory_los, c_origins, c_radius, settings, static_obs_collection)
persistent graph_handles
persistent graph_handles2
persistent counter
persistent dontprintagain


    if isempty(graph_handles)
        graph_handles = [];
        graph_handles2 = [];
        counter = 0;
        dontprintagain = 0;
    end
    
    OS_Scale = 1;
    
    t = loopdata(:,1);
    xref_N = loopdata(:,2);
    xref_E = loopdata(:,3);
    psi_ref = loopdata(:,4);
    surge_ref = loopdata(:,5);
    sway_ref = loopdata(:,6);
    r_ref = loopdata(:,7);
    
    north_opt = w_opt(1:9:end);
    east_opt = w_opt(2:9:end);
    psi_opt = w_opt(3:9:end);
    surge_opt = w_opt(4:9:end);
    sway_opt = w_opt(5:9:end);
    r_opt = w_opt(6:9:end);
    Fx_opt = w_opt(7:9:end);
    Fy_opt = w_opt(8:9:end);
    Fn_opt = w_opt(9:9:end);
    
    N_error = north_opt - xref_N;
    E_error = east_opt - xref_E;
    
    if~isempty(static_obs_collection)
        [~,c] = find(isnan(static_obs_collection(1,:)));
        static_obs = get_global_map_data();
    end

%     figure(600);
%     previous_linex = [];
%     previous_liney = [];
%     for k = 1:c(1)-1
%         pos = static_obs_collection(1:2,k);
%         ang = static_obs_collection(3,k);
%         dir = [cos(ang);sin(ang)];
%         point = pos + 1000*dir;
%         point2 = pos - 1000*dir;
%         linex = [point2(2), point(2), NaN];
%         liney = [point2(1), point(1), NaN];
%         
%         if ~isempty(previous_linex)
%             [xi, yi] = polyxpoly(linex, liney, previous_linex, previous_liney);
%             linex = [xi(1), point2(2), NaN];
%             liney = [yi(1), point2(1), NaN];
%         end
%         previous_linex = [previous_linex, linex];
%         previous_liney = [previous_liney, liney];
%     end
%     delete(static_obs_handles);
%     h1 = mapshow(previous_linex, previous_liney,'linewidth',1.2);
%     h2 = mapshow(static_obs(2,:),static_obs(1,:),'DisplayType','polygon','LineStyle','none');
%     static_obs_handles = [static_obs_handles; h1; h2];

    
     
    
    figure(10);
    clf;
    subplot(3,1,1);
    hold on;
    plot(t,xref_N);
    plot(t,north_opt,'*');
    hold off;
    grid;
    title('North ref and North Opt');
    xlabel('Discretized time [k]');
    ylabel('North [m]');
    legend('North ref','North Opt');
    
    subplot(3,1,2);
    hold on;
    plot(t,xref_E);
    plot(t,east_opt,'*');
    hold off;
    grid;
    title('East ref and East opt');
    xlabel('Discretized Time [k]');
    ylabel('East [m]');
    legend('East ref','East opt');
    
    subplot(3,1,3);
    hold on;
    plot(t,psi_ref);
    plot(t,psi_opt,'*');
    hold off;
    grid;
    title('psi ref and psi opt');
    xlabel('Discretized time [k]');
    ylabel('Psi (rad)');
    legend('Psi ref','Psi opt');
    
    
    figure(11);
    clf;
    subplot(3,1,1);
    hold on;
    plot(t,surge_ref);
    plot(t,surge_opt,'*');
    hold off;     
    grid;
    title('surge ref and surge Opt');
    xlabel('Discretized time [k]');
    ylabel('Surge [m/s]');
    legend('Surge ref','Surge Opt');

    subplot(3,1,2);
    hold on;
    plot(t,sway_ref);     
    plot(t,sway_opt,'*');
    hold off;     
    grid;
    title('sway ref and sway Opt');
    xlabel('Discretized time [k]');
    ylabel('sway [m/s]');
    legend('sway ref','sway Opt');

    subplot(3,1,3);
    hold on;
    plot(t,r_ref);     
    plot(t,r_opt,'*');
    hold off;
    grid;
    title('yaw rate ref and yaw rate Opt');
    xlabel('Discretized time [k]');
    ylabel('yaw rate [rad/s]');
    legend('Yaw rate ref','Yaw Rate Opt');
    
    
    figure(12);
    clf;
    subplot(311)
    plot(t(1:end-1),Fx_opt,'*');
    grid;
    title('Optimal Force Fx');
    xlabel('Disctretised time [k]');
    ylabel('Force [N]');
    legend('Fx');
    subplot(312)
    plot(t(1:end-1),Fy_opt,'*');
    grid;
    title('Optimal Force Fy');
    xlabel('Disctretised time [k]');
    ylabel('Force [N]');
    legend('Fy');
    subplot(313)
    plot(t(1:end-1),Fn_opt,'*');
    grid;
    title('Optimal Force Fn');
    xlabel('Disctretised time [k]');
    ylabel('Force [N]');
    legend('Fn');
     
    figure(13);
    clf;
    subplot(211)
    plot(t,N_error);
    grid;
    title('Positional error in North');
    xlabel('Discretized time step [k]');
    ylabel('error in meters [m]');
    subplot(212);
    plot(t,E_error);
    grid;
    title('Positional error in East');
    xlabel('Discretized time step [k]');
    ylabel('error in meters [m]');
       
    figure(1);
    delete(graph_handles)
%     clf;
%     xaxis = [vessel.eta(2) - 200, vessel.eta(2) + 200];
%     yaxis = [vessel.eta(1) - 200, vessel.eta(1) + 200];
%     axis([xaxis, yaxis]); % FOR BIG SIMS
%     graph_index = 1;
    axis([settings.axis])
%     if isempty(graph_handles)
%         grid;
%     end
    hold on
    for j = 1:size(tracks,2)
    agent_eta = [tracks(j).eta(1:2,1);atan2(tracks(j).eta_dot(2,1), tracks(j).eta_dot(1,1))];
    handle_TS = plot_os(agent_eta, 'r', OS_Scale); % Eta
    handle_TS_Quiver = quiver(tracks(j).eta(2), tracks(j).eta(1), tracks(j).eta_dot(2),tracks(j).eta_dot(1),10,'r','filled');
    graph_handles = [graph_handles, handle_TS];
    graph_handles = [graph_handles, handle_TS_Quiver];
    if counter > 2
        plot_trajectory(tracks(j),'red'); % persistent_TS_traj
    end

    end
    agent_eta = [vessel.eta(1:2,1);atan2(vessel.eta_dot(2,1), vessel.eta_dot(1,1))];
    handle_OS = plot_os(agent_eta, 'b', OS_Scale); % Eta
    handle_OS_Quiver = quiver(agent_eta(2), agent_eta(1), vessel.eta_dot(2),vessel.eta_dot(1),10,'b','filled');
    graph_handles = [graph_handles, handle_OS];
    graph_handles = [graph_handles, handle_OS_Quiver];
    if counter > 2
        plot_trajectory(vessel,'blue'); % persistent_OS_traj  
        counter = 0;
    end
%     plot_trajectory(tracks(j),'blue');
    
    if~isempty(c_radius)
        for i = 1:min(10,length(c_radius))
            th = 0:pi/50:2*pi;
            xunit = c_radius(i) * cos(th) + c_origins(2,i);
            yunit = c_radius(i) * sin(th) + c_origins(1,i);
            handle_D_constraints = plot(xunit,yunit,'r');
            graph_handles = [graph_handles, handle_D_constraints];
        end
    end
    if ~isempty(static_obs_collection)
        previous_linex = [];
        previous_liney = [];
        for k = 1:c(1)-1
            pos = static_obs_collection(1:2,k);
            ang = static_obs_collection(3,k);
            dir = [cos(ang);sin(ang)];
            point = pos + 1000*dir;
            point2 = pos - 1000*dir;
            linex = [point2(2), point(2), NaN];
            liney = [point2(1), point(1), NaN];

%             if ~isempty(previous_linex)
%                 [xi, yi] = polyxpoly(linex, liney, previous_linex, previous_liney);
%                 if ~isempty(xi)
%                     linex = [xi(1), point2(2), NaN];
%                     liney = [yi(1), point2(1), NaN];
%                 else
%                     linex = [NaN, NaN, NaN];
%                     liney = [NaN, NaN, NaN];
%                 end
%             end
            previous_linex = [previous_linex, linex];
            previous_liney = [previous_liney, liney];
        end
        handle_S_constraints = mapshow(previous_linex, previous_liney,'linewidth',1.2);
        graph_handles = [graph_handles, handle_S_constraints];
        if ~dontprintagain
            mapshow(static_obs(2,:),static_obs(1,:),'DisplayType','polygon','LineStyle','-','FaceColor',get_rgb('land'),'EdgeColor','black','LineWidth',1);
        end
    end
    hold off;
    xlabel('East [m]');
    ylabel('North [m]');
    title('Simulation with constraint circles');

    tic;
    figure(999);
    delete(graph_handles2)
    axis(settings.axis);
    hold on; 
    %plot trajectories
    handle_wopt = plot(east_opt, north_opt, 'b*','LineWidth',0.3);
    handle_path =  plot(vessel.wp(2,:),vessel.wp(1,:),'g');
    handle_reftraj = plot(reference_trajectory_los(2,:),reference_trajectory_los(1,:) , 'r--');
    graph_handles2 = [graph_handles2, handle_wopt, handle_path, handle_reftraj];
    %plot constraint circles
%     if~isempty(c_radius)
%         for i = 1:10
%             th = 0:pi/50:2*pi;
%             xunit = c_radius(i) * cos(th) + c_origins(2,i);
%             yunit = c_radius(i) * sin(th) + c_origins(1,i);
%             plot(xunit,yunit);
%         end
%     end
    %plot constraint centers
    if ~isempty(c_origins)
        handle_D_constraints = plot(c_origins(2,:),c_origins(1,:),'rx'); 
        graph_handles2 = [graph_handles2, handle_D_constraints];
    end
    %lables
    handle_OS_w = plot_os(agent_eta, 'b', OS_Scale); % Eta
    graph_handles2 = [graph_handles2, handle_OS_w];
    for j = 1:size(tracks,2)
        agent_eta = [tracks(j).eta(1:2,1);atan2(tracks(j).eta_dot(2,1), tracks(j).eta_dot(1,1))];
        handle_TS_w = plot_os(agent_eta, 'r', OS_Scale); % Eta
        handle_TS_Quiver = quiver(tracks(j).eta(2), tracks(j).eta(1), tracks(j).eta_dot(2),tracks(j).eta_dot(1),10,'r','filled');
        graph_handles2 = [graph_handles2, handle_TS_w, handle_TS_Quiver];
    end
    if ~isempty(static_obs_collection) && ~dontprintagain
        mapshow(static_obs(2,:),static_obs(1,:),'DisplayType','polygon','LineStyle','-','FaceColor',get_rgb('land'),'EdgeColor','black','LineWidth',1);
        dontprintagain = 1;
    end
    
    hold off;
    title('Projected future trajectory');
    xlabel('East [m]');
    ylabel('North [m]');
    legend('off')
    if ~isempty(c_origins)
        legend('','W_{opt}', 'Transit path', 'reference trajectory','Dynamic Constraint origin');
    else
        if dontprintagain
        legend('','W_{opt}', 'Transit path', 'reference trajectory','');
        else
        legend('W_{opt}', 'Transit path', 'reference trajectory','');
        end
    end
    toc


    plots = [];
    counter = counter + 1;
end