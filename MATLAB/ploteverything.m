function plots = ploteverything(loopdata,w_opt, vessel, tracks, reference_trajectory_los, c_origins, c_radius, settings, static_obs_collection)
persistent graph_handles
persistent graph_handles2
persistent counter
persistent dontprintagain
persistent TS_traj_handle
persistent OS_traj_handle


    if isempty(graph_handles)
        graph_handles = gobjects(50,1);
        graph_handles2 = gobjects(50,1);
        counter = 0;
        dontprintagain = 0;
        TS_traj_handle = [];
        OS_traj_handle = [];
        %handle counters:

    end

    %% COLORS
%     w_opt_color = [60, 171, 77]./256;
%     path_color = [210, 37, 230]./256;
%     ref_color = [37, 130, 230]./256;
%     static_obs_color = [217, 35, 83]./256;
    sea = [184, 244, 255]./256;
    w_opt_color = [209, 27, 250]./256;
    path_color = [0, 207, 107]./256;
    ref_color = [255, 42, 0]./256;
    static_obs_color = [255, 22, 93]./256;
    %%
    
    OS_Scale = 1;
    legend_size_H = 0;
    HC1 = 1; 
    HC2 = 1;
    
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
    handle_TS = plot_ts_from_struct(tracks(j), 'r', OS_Scale); % Eta
    handle_TS_Quiver = quiver(tracks(j).eta(2), tracks(j).eta(1), tracks(j).eta_dot(2),tracks(j).eta_dot(1),10,'r','filled');
    graph_handles(HC1) = handle_TS;
    HC1 = HC1 + 1;
    graph_handles(HC1) = handle_TS_Quiver;
    HC1 = HC1 + 1;
    if counter > 2
        TS_traj_handle = plot_trajectory(tracks(j),'red'); % persistent_TS_traj
    end
    if j == 1
        legend_size_H = legend_size_H + 60;
    end

    end
    agent_eta = [vessel.eta(1:2,1);atan2(vessel.eta_dot(2,1), vessel.eta_dot(1,1))];
    handle_OS = plot_os(agent_eta, 'b', OS_Scale); % Eta
    handle_OS_Quiver = quiver(agent_eta(2), agent_eta(1), vessel.eta_dot(2),vessel.eta_dot(1),10,'b','filled');
    graph_handles(HC1) = handle_OS;
    HC1 = HC1 + 1;
    graph_handles(HC1) = handle_OS_Quiver;
    HC1 = HC1 + 1;
    if counter > 2
        OS_traj_handle = plot_trajectory(vessel,'blue'); % persistent_OS_traj  
        counter = 0;
    end
    legend_size_H = legend_size_H + 60;
%     plot_trajectory(tracks(j),'blue');
    
    if~isempty(c_radius)
        for i = 1:min(10,length(c_radius))
            th = 0:pi/50:2*pi;
            xunit = c_radius(i) * cos(th) + c_origins(2,i);
            yunit = c_radius(i) * sin(th) + c_origins(1,i);
            handle_D_constraints = plot(xunit,yunit,'r');
            graph_handles(HC1) = handle_D_constraints;
            HC1 = HC1 + 1;
        end
        legend_size_H = legend_size_H + 30;
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
        handle_S_constraints = mapshow(previous_linex, previous_liney,'linewidth',1.2,'Color',static_obs_color);
        graph_handles(HC1) = handle_S_constraints;
        HC1 = HC1 + 1;
        if ~isempty(previous_linex)
            legend_size_H = legend_size_H + 30;
        end
        if ~dontprintagain
            mapshow(static_obs(2,:),static_obs(1,:),'DisplayType','polygon','LineStyle','-','FaceColor',get_rgb('land'),'EdgeColor','black','LineWidth',1);
        end
        legend_size_H = legend_size_H + 30;
    end

    
    scale = (settings.axis(2)-settings.axis(1))/760*0.8;
    legend_size = [305,legend_size_H]*scale;
    legend_position = settings.legend_position;

    legend_item_height_offset = 15;

    handle_whitesquare = rectangle('Position',[legend_position(2), legend_position(1),legend_size(1),legend_size(2)],'FaceColor',[1 1 1],'EdgeColor',[0 0 0],'LineWidth',1);
    vessel.eta(1:2) = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
    vessel.eta(3,1) = -pi/2;
    handle_OS_legend = plot_os_from_struct(vessel, 'b', 4.0*scale);
    thandle_OS = text(vessel.eta(2)+scale*25, vessel.eta(1), 'Own ship', 'fontsize',13);
    graph_handles(HC1) = handle_whitesquare;
    HC1 = HC1 + 1;
    graph_handles(HC1) = handle_OS_legend;
    HC1 = HC1 + 1;
    graph_handles(HC1) = thandle_OS;
    HC1 = HC1 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    if ~isempty(tracks)
        vessel.eta(1:2) = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        vessel.eta(3,1) = -pi/2;
        handle_ = plot_ts_from_struct(vessel, 'r' ,4.0*scale);
        thandle = text(vessel.eta(2)+scale*25, vessel.eta(1), 'Target ships', 'fontsize',13);
        graph_handles(HC1) = handle_;
        HC1 = HC1 + 1;
        graph_handles(HC1) = thandle;
        HC1 = HC1 + 1;
        legend_item_height_offset = legend_item_height_offset + 30;
    end

    line_delta = [-30*scale;0];
    line_start = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*10];
    line_mid1 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*20];
    line_mid2 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*30];
    line_end   = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*40];
    line = [line_start,line_mid1,line_mid2,line_end];
    handle_ = plot(line(2,:), line(1,:),'b--', 'linewidth',2);
    thandle = text(line_end(2)+10*scale, line_start(1), 'Own ship trajectory', 'fontsize',13);
    graph_handles(HC1) = handle_;
    HC1 = HC1 + 1;
    graph_handles(HC1) = thandle;
    HC1 = HC1 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    if ~isempty(tracks)
        line_start = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*10];
        line_mid1 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*20];
        line_mid2 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*30];
        line_end   = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*40];
        line = [line_start,line_mid1,line_mid2,line_end];
        handle_ = plot(line(2,:), line(1,:),'r--', 'linewidth',2);
        thandle = text(line_end(2)+10*scale, line_start(1), 'Target ship trajectory', 'fontsize',13);
        graph_handles(HC1) = handle_;
        HC1 = HC1 + 1;
        graph_handles(HC1) = thandle;
        HC1 = HC1 + 1;  
        legend_item_height_offset = legend_item_height_offset + 30;
    end

    if ~isempty(static_obs_collection)
        position = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        handle_ = legend_filled_square(position, 30*scale, get_rgb('land'));
        thandle = text(position(2) + scale*25, position(1), 'Static obstacles', 'fontsize',13);
        graph_handles(HC1) = handle_;
        HC1 = HC1 + 1;
        graph_handles(HC1) = thandle;
        HC1 = HC1 + 1; 
        legend_item_height_offset = legend_item_height_offset + 30;
    end
    if ~isempty(static_obs_collection) && ~isempty(previous_linex)
        position = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        handle_ =  legend_filled_rectangle(position, 30*scale, static_obs_color);
        thandle = text(position(2) + scale*25, position(1), 'Static obstacles constraint', 'fontsize',13);
        graph_handles(HC1) = handle_;
        HC1 = HC1 + 1;
        graph_handles(HC1) = thandle;
        HC1 = HC1 + 1; 
        legend_item_height_offset = legend_item_height_offset + 30;
    end

    if ~isempty(c_radius)
        th = 0:pi/20:2*pi;
        position = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        xunit = 3 * cos(th) + position(2);
        yunit = 3 * sin(th) + position(1);
        handle_ = plot(xunit,yunit,'r');
        thandle = text(position(2)+scale*25, position(1), 'Dynamic obstacles constraint (first 10)', 'fontsize',13);
        graph_handles(HC1) = handle_;
        HC1 = HC1 + 1;
        graph_handles(HC1) = thandle;
    end


    hold off;
    xlabel('East [m]','FontSize',23);
    ylabel('North [m]','FontSize',23);
    set(gca,'color',sea);
%     % Make legend vector:
%     leg_vekt = [handle_OS];
%     
%     if ~isempty(handle_D_constraints)
%         leg_vekt = [leg_vect, handle_D_constraints]
%     end

%     legend([TS_traj_handle handle_TS handle_OS OS_traj_handle handle_D_constraints handle_S_constraints], ...
%         {'Target Ship Transit' , 'Target Ship', 'Own Ship', 'Own Ship Transit', 'Dynamic Constraint Circle', 'Static Constraint Line'}    
%     legend_vector = [handle_OS];
%     legend_string = {'Own Ship'};
%     legend(legend_vector,legend_string)
%     title('Simulation with constraint circles');

    figure(999);
    delete(graph_handles2)
    axis(settings.axis);
    hold on; 
    %plot trajectories
    handle_path =  plot(vessel.wp(2,:),vessel.wp(1,:),'color',path_color,'LineStyle','-', 'linewidth',2,'Marker','none');
    handle_reftraj = plot(reference_trajectory_los(2,:),reference_trajectory_los(1,:), ...
        'color',ref_color,'LineStyle','--', 'linewidth',2,'Marker','none','MarkerSize',2,'MarkerFaceColor',ref_color,'MarkerIndices',1:2:length(reference_trajectory_los(1,:)));
    handle_wopt = plot(east_opt, north_opt,'color',w_opt_color,'LineStyle','none', 'MarkerSize',5,'Marker','o','MarkerFaceColor',w_opt_color,'MarkerIndices',1:2:length(north_opt));
    graph_handles2(HC2) = handle_wopt;
    HC2 = HC2 + 1; 
    graph_handles2(HC2) = handle_path;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = handle_reftraj;
    HC2 = HC2 + 1;
    legend_size_H = 90;
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
        graph_handles2(HC2) = handle_D_constraints;
        HC2 = HC2 + 1;
        legend_size_H = legend_size_H + 30;
    end
    %lables
    handle_OS_w = plot_os(agent_eta, 'b', OS_Scale); % Eta
    graph_handles2(HC2) = handle_OS_w;
    HC2 = HC2 + 1;
    legend_size_H = legend_size_H + 30;
    for j = 1:size(tracks,2)
        handle_TS_w = plot_ts_from_struct(tracks(j), 'r', OS_Scale); % Eta
        handle_TS_Quiver = quiver(tracks(j).eta(2), tracks(j).eta(1), tracks(j).eta_dot(2),tracks(j).eta_dot(1),10,'r','filled');
        graph_handles2(HC2) = handle_TS_w;
        HC2 = HC2 + 1;
        graph_handles2(HC2) =  handle_TS_Quiver;
        HC2 = HC2 + 1;
        if j == 1
            legend_size_H = legend_size_H + 30;
        end
    end
    if ~isempty(static_obs_collection) && ~dontprintagain
        mapshow(static_obs(2,:),static_obs(1,:),'DisplayType','polygon','LineStyle','-','FaceColor',get_rgb('land'),'EdgeColor','black','LineWidth',1);
        dontprintagain = 1;
    end
    if ~isempty(static_obs_collection)
        legend_size_H = legend_size_H + 30;
    end

    scale = (settings.axis(2)-settings.axis(1))/760*0.8;
    legend_size = [250,legend_size_H]*scale;
    legend_position = settings.legend_position;

    legend_item_height_offset = 15;

    handle_whitesquare = rectangle('Position',[legend_position(2), legend_position(1),legend_size(1),legend_size(2)],'FaceColor',[1 1 1],'EdgeColor',[0 0 0],'LineWidth',1);

    % wopt, ref, path:
    line_delta = [-30*scale;0];
    line_start = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*10];
    line_mid1 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*20];
    line_mid2 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*30];
    line_end   = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*40];
    line = [line_start,line_mid1,line_mid2,line_end];
    handle_ = plot(line(2,:), line(1,:),'color',w_opt_color,'LineStyle','none', 'MarkerSize',5,'Marker','o','MarkerFaceColor',w_opt_color);
    thandle_ = text(line_end(2)+10*scale, line_start(1), 'Optimal trajectory', 'fontsize',13);
    graph_handles2(HC2) = handle_;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = thandle_;
    HC2 = HC2 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    line_start = line_start + line_delta;
    line_end = line_end + line_delta;
    line = [line_start, line_end]; 
    handle_ = plot(line(2,:), line(1,:),'color',path_color,'LineStyle','-', 'linewidth',2,'Marker','none');
    thandle_ = text(line_end(2)+10*scale, line_start(1), 'Reference path ', 'fontsize',13);
    graph_handles2(HC2) = handle_;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = thandle_;
    HC2 = HC2 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    line_start = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*10];
    line_mid1 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*20];
    line_mid2 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*30];
    line_end   = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*40];
    line = [line_start,line_mid1,line_mid2,line_end];
    handle_ = plot(line(2,:), line(1,:),'color',ref_color,'LineStyle','--', 'linewidth',2);
    thandle_ = text(line_end(2)+10*scale, line_start(1), 'Reference trajectory', 'fontsize',13);
    graph_handles2(HC2) = handle_;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = thandle_;
    HC2 = HC2 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    vessel.eta(1:2) = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
    vessel.eta(3,1) = -pi/2;
    handle_OS_legend = plot_os_from_struct(vessel, 'b', 4.0*scale);
    thandle_OS = text(vessel.eta(2)+scale*25, vessel.eta(1), 'Own ship', 'fontsize',13);
    graph_handles2(HC2) = handle_whitesquare;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = handle_OS_legend;
    HC2 = HC2 + 1;
    graph_handles2(HC2) = thandle_OS;
    HC2 = HC2 + 1;
    legend_item_height_offset = legend_item_height_offset + 30;

    if ~isempty(tracks)
        vessel.eta(1:2) = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        vessel.eta(3,1) = -pi/2;
        handle_ = plot_ts_from_struct(vessel, 'r' ,4.0*scale);
        thandle = text(vessel.eta(2)+scale*25, vessel.eta(1), 'Target ships', 'fontsize',13);
        graph_handles2(HC2) = handle_;
        HC2 = HC2 + 1;
        graph_handles2(HC2) = thandle;
        HC2 = HC2 + 1;
        legend_item_height_offset = legend_item_height_offset + 30;
    end

    if ~isempty(static_obs_collection)
        position = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*25];
        handle_ = legend_filled_square(position, 30*scale, get_rgb('land'));
        thandle = text(position(2) + scale*25, position(1), 'Static obstacles', 'fontsize',13);
        graph_handles2(HC2) = handle_;
        HC2 = HC2 + 1;
        graph_handles2(HC2) = thandle;
        HC2 = HC2 + 1;
        legend_item_height_offset = legend_item_height_offset + 30;
    end

    if ~isempty(c_radius)
        line_start = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*10];
        line_mid1 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*20];
        line_mid2 = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*30];
        line_end   = legend_position + [legend_size(2)-legend_item_height_offset*scale; scale*40];
        line = [line_start,line_mid1,line_mid2,line_end];
        handle_ = plot(line(2,:), line(1,:),'rx', 'linewidth',1);
        thandle = text(line_end(2)+10*scale, line_start(1), 'Dynamic constraint origin', 'fontsize',13);
        graph_handles2(HC2) = handle_;
        HC2 = HC2 + 1;
        graph_handles2(HC2) = thandle;
    end
    
    hold off;
%     title('Projected future trajectory');
    xlabel('East [m]','FontSize',23);
    ylabel('North [m]','FontSize',23);
    set(gca,'color',sea);

    plots = [];
    counter = counter + 1;
end