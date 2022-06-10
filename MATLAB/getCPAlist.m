function [dCPAlist, tCPAlist, pos_OS_list, pos_TS_list] = getCPAlist(vessel,tracks)
dCPAlist = [];
tCPAlist = [];
pos_TS_list = [];
pos_OS_list = [];
wptstimer = 0; % timer used to calculate the position of the other ship at certain wpts.

%%

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

    %Find pose at current OS waypoint
    [pos_OS, vel_OS] = VesselWPReadout(vessel,i);
    [pos_TS, vel_TS] = whereisTS(tracks,wptstimer); 

    heading_OS = atan2(vel_OS(2),vel_OS(1));
    heading_TS = atan2(vel_TS(2),vel_TS(1));

    %Do cpa check
    [dCPA, tCPA] = ClosestApproach(pos_OS, pos_TS, vel_OS, vel_TS);
    dCPA = round(dCPA*1000)/1000;
    tCPA = tCPA + wptstimer; % håper dette blir rett.
    pos_OS = [pos_OS;heading_OS];
    pos_TS = [pos_TS;heading_TS];
    pos_OS_list = [pos_OS_list, pos_OS];
    pos_TS_list = [pos_TS_list, pos_TS];


    %step forward one waypoint.
    distancetonextWP = sqrt((vessel.wp(1,i+1) - pos_OS(1))^2 + ((vessel.wp(2,i+1) - pos_OS(2))^2));
    timetonextWP = distancetonextWP / norm(vessel.nu(1:2),2); %Distanse OG time to next wp er redundant, egentlig kunne jeg klart meg med en.
    wptstimer = wptstimer + timetonextWP;
    dCPAlist = [dCPAlist, dCPA];
    tCPAlist = [tCPAlist, tCPA];

  
end
end