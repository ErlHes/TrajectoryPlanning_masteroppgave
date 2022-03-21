function [flag, dCPA, tCPA] = COLREGs_assessment(vessel,tracks, cflag,simple)
% a13 = 112.5; % Overtaking tolerance
% a14 = rad2deg(pi/8); % head-on tolerance
% a15 = rad2deg(pi/8); % crossing aspect limit

%% Angles between OS and TS
phi_1 = rad2deg(pi/8);
phi_2 = 112.5;

b0 = rad2deg(wrapTo2Pi(atan2(tracks.eta(2)-vessel.eta(2),tracks.eta(1)-vessel.eta(1))) - ssa(vessel.eta(3))); % Relative from OS to TS
b0_180 = ssa(b0, 'deg');

a0 = rad2deg(ssa(atan2(vessel.eta(2)-tracks.eta(2),vessel.eta(1)-tracks.eta(1)) - tracks.eta(3))); % Relative from TS to OS

% dist = sqrt((tracks.eta(2) - vessel.eta(2))^2 + (tracks.eta(1) - vessel.eta(1))^2);
%a0_360 = rad2deg(wrapTo2Pi(deg2rad(a0)));
% 
% 
% phi_TS = atan2((vessel.eta(2)-tracks.eta(2)), (vessel.eta(1) - tracks.eta(1)));
% psi_TSR = tracks.eta(3) - vessel.eta(3) - phi_TS;
% 
% phi_TS = wrapTo2Pi(phi_TS);
% psi_TSR = wrapTo2Pi(psi_TSR);

%% Calculate dCPA and tCPA, check if COLREGs assessment is needed:
% [dCPA, tCPA] = ClosestApproach(vessel.eta(1:2), tracks.eta(1:2), vessel.eta_dot(1:2), tracks.eta_dot(1:2));
dCPAlist = [];
tCPAlist = [];
wptstimer = 0;
for i = vessel.current_wp:size(vessel.wp,2)-1
            % NAIV APPROACH
% % % % % % % %     %For each OS transit waypoint, check the dCPA and tCPA for each TS trasit
% % % % % % % %     %waypoint.
% % % % % % % %     [pos_OS, vel_OS] = VesselReadout(vessel,i);
% % % % % % % %     for j = tracks.current_wp:size(tracks.wp,2)-1
% % % % % % % %         [pos_TS, vel_TS] = VesselReadout(tracks,j);
% % % % % % % %         [dCPA, tCPA] = ClosestApproach(pos_OS, pos_TS, vel_OS, vel_TS);
% % % % % % % %         dCPAlist(i,j) = dCPA;
% % % % % % % %         tCPAlist(i,j) = tCPA;
% % % % % % % %     end
            % NAIV APPROACH ^

%Fra vessel.eta, hvor lang tid tar det å nå neste wpt?
%Fra neste wpt, hvor lang tid tar det å nå neste wpt? <- repeat for alle
%wpts. Anta konstant fart hele veien.
    [pos_OS, vel_OS] = VesselWPReadout(vessel,i);
    distancetonextWP = sqrt((vessel.wp(1,i+1) - pos_OS(1))^2 + ((vessel.wp(2,i+1) - pos_OS(2))^2));
    timetonextWP = distancetonextWP / norm(vessel.nu(1:2),2);
    [pos_TS, vel_T] = whereisTS(tracks,wptstimer,timetonextWP); % Find where TS is when OS is at current position

    %%TODO: Skjekk *alle* relevante TS waypoints når OS følger en veldig
    %%lang bane.

    wptstimer = wptstimer + timetonextWP;
    [dCPA, tCPA] = ClosestApproach(pos_OS, pos_TS, vel_OS, vel_TS);
    tCPA = tCPA + wptstimer; % Blir dette rett? hvem vet.
    dCPAlist = [dCPAlist, dCPA];
    tCPAlist = [tCPAlist, tCPA];

  
end
%Keep the lowest dCPA found, this is the only dCPA we're interested in
[dCPA, CPAindex] = min(dCPAlist);
tCPA = tCPAlist(CPAindex);




%% Conduct COLREGs assessment
% 1 = HO
% 2 = GW
% 3 = SO
% 4 = OT
% 5 = SF
if cflag == 0 %%
    if abs(b0_180) < phi_1 % TS is direcly ahead of OS
        if abs(a0) < phi_1 % TS is facing OS
            flag = 1;
        elseif a0 > phi_1 && a0 < phi_2 % TS is facing towards OS's starboard
            flag = 3;
        elseif a0 < (-phi_1) && a0 > (-phi_2) % TS is facing towards OS's port
            flag = 2;
        else                            % TS is facing away from OS
            flag = 4; 
        end
    elseif b0 > phi_1 && b0 < phi_2 %TS is ahead on OS's starboard
        if abs(a0) < phi_1
            flag = 2;   
        elseif a0 > phi_1 && a0 < phi_2
            flag = 5;
        elseif a0 < (-phi_1) && a0 > (-phi_2)
            flag = 2;
        else
            flag = 4;
        end
    elseif b0_180 < -phi_1 && b0_180 > -phi_2 %TS is ahead on OS's port side
        if abs(a0) < phi_1
            flag = 3;   
        elseif a0 > phi_1 && a0 < phi_2
            flag = 3;
        elseif a0 < (-phi_1) && a0 > (-phi_2)
            flag = 5;
        else
            flag = 4;
        end
    else
        if abs(a0) < phi_1
            flag = 3;   
        elseif a0 > phi_1 && a0 < phi_2
            flag = 3;
        elseif a0 < (-phi_1) && a0 > (-phi_2)
            flag = 3;
        else
            flag = 5;
        end    
    end
else % hackjob, needs more work to clear situations properly.
flag = cflag;
end
    

%% Woerner method
% if b0 > 112.5 && b0 < 247.5 && abs(a0) < a13
%     flag = 'SO';
% elseif a0_360 > 112.5 && a0_360 < 247.5 && abs(b0_180) < a13,
%     flag = 'GW';
% elseif abs(b0_180) < a14 && abs(a0) < a14
%     flag = 'HO';
% elseif b0 > 0 && b0 < 112.5 && a0 > -112.5 && a0 < a15
%     flag = 'GW';
% elseif a0_360 > 0 && a0_360 < 112.5 && b0_180 < -112.5 && b0_180 < a15
%     flag = 'SO';
%     
% else
%     flag = 'SO';
end