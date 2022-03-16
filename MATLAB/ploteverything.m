function plots = ploteverything(loopdata,w_opt, vessel, tracks, reference_trajectory_los, c_origins, c_radius, settings)
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
    

    figure(999);
    clf;
    axis(settings.axis);
    hold on;
    %plot trajectories
    plot(east_opt, north_opt, '*');
    plot(vessel.wp(2,:),vessel.wp(1,:),'g');
    plot(reference_trajectory_los(2,:),reference_trajectory_los(1,:) , 'r-.');
    %plot constraint circles
    if~isempty(c_radius)
        for i = 1:10
            th = 0:pi/50:2*pi;
            xunit = c_radius(i) * cos(th) + c_origins(2,i);
            yunit = c_radius(i) * sin(th) + c_origins(1,i);
            plot(xunit,yunit);
        end
    end
    %lables
    title('Projected future trajectory');
    xlabel('East [m]');
    ylabel('North [m]');
    legend('W_{opt}', 'Transit path', 'reference trajectory');
    grid;
    
    figure(10);
    clf;
    subplot(3,1,1);
    plot(t,xref_N);
    hold on;
    plot(t,north_opt,'*');
    hold off;
    grid;
    title('North ref and North Opt');
    xlabel('Discretized time [k]');
    ylabel('North [m]');
    legend('North ref','North Opt');
    
    subplot(3,1,2);
    plot(t,xref_E);
    hold on;
    plot(t,east_opt,'*');
    hold off;
    grid;
    title('East ref and East opt');
    xlabel('Discretized Time [k]');
    ylabel('East [m]');
    legend('East ref','East opt');
    
    subplot(3,1,3);
    plot(t,psi_ref);
    hold on;
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
    plot(t,surge_ref);
    hold on;
    plot(t,surge_opt,'*');
    hold off;
    grid;
    title('surge ref and surge Opt');
    xlabel('Discretized time [k]');
    ylabel('Surge [m/s]');
    legend('Surge ref','Surge Opt');

    subplot(3,1,2);
    plot(t,sway_ref);
    hold on;
    plot(t,sway_opt,'*');
    hold off;
    grid;
    title('sway ref and sway Opt');
    xlabel('Discretized time [k]');
    ylabel('sway [m/s]');
    legend('sway ref','sway Opt');

    subplot(3,1,3);
    plot(t,r_ref);
    hold on;
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
    clf;
    axis(settings.axis);
    grid;
    hold on
    for j = 1:size(tracks,2)
    agent_eta = [tracks(j).eta(1:2,1);atan2(tracks(j).eta_dot(2,1), tracks(j).eta_dot(1,1))];
    plot_os(agent_eta, 'r', 2); % Eta
    end
    agent_eta = [vessel.eta(1:2,1);atan2(vessel.eta_dot(2,1), vessel.eta_dot(1,1))];
    plot_os(agent_eta, 'b', 2); % Eta
    if~isempty(c_radius)
        for i = 1:10
            th = 0:pi/50:2*pi;
            xunit = c_radius(i) * cos(th) + c_origins(2,i);
            yunit = c_radius(i) * sin(th) + c_origins(1,i);
            plot(xunit,yunit);
        end
    end
    xlabel('East [m]');
    ylabel('North [m]');
    title('Simulation with constraint circles');
    
    plots = 0;