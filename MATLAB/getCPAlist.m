function [dCPAlist, tCPAlist] = getCPAlist(vessel,tracks)
dCPAlist = [];
tCPAlist = [];
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
    [pos_OS, vel_OS] = VesselWPReadout(vessel,i);
    distancetonextWP = sqrt((vessel.wp(1,i+1) - pos_OS(1))^2 + ((vessel.wp(2,i+1) - pos_OS(2))^2));
    timetonextWP = distancetonextWP / norm(vessel.nu(1:2),2); %Distanse OG time to next wp er redundant, egentlig kunne jeg klart meg med en.
    [pos_TS, vel_TS] = whereisTS(tracks,wptstimer,timetonextWP); % Find where TS is when OS is at current position

    %%TODO: Skjekk *alle* relevante TS waypoints når OS følger en veldig
    %%lang bane.

    [dCPA, tCPA] = ClosestApproach(pos_OS, pos_TS, vel_OS, vel_TS);
    dCPA = round(dCPA*1000)/1000;
    tCPA = tCPA + wptstimer; % Blir dette rett? hvem vet.

    wptstimer = wptstimer + timetonextWP;
    dCPAlist = [dCPAlist, dCPA];
    tCPAlist = [tCPAlist, tCPA];

  
end
end