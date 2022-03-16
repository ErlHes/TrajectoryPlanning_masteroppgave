function flag = COLREGs_assessment(vessel,tracks, cflag)
% a13 = 112.5; % Overtaking tolerance
% a14 = rad2deg(pi/8); % head-on tolerance
% a15 = rad2deg(pi/8); % crossing aspect limit

phi_1 = rad2deg(pi/8);
phi_2 = 112.5;

b0 = rad2deg(wrapTo2Pi(atan2(tracks.eta(2)-vessel.eta(2),tracks.eta(1)-vessel.eta(1))) - ssa(vessel.eta(3))); % Relative from OS to TS
b0_180 = ssa(b0, 'deg');

a0 = rad2deg(ssa(atan2(vessel.eta(2)-tracks.eta(2),vessel.eta(1)-tracks.eta(1)) - tracks.eta(3))); % Relative from TS to OS

dist = sqrt((tracks.eta(2) - vessel.eta(2))^2 + (tracks.eta(1) - vessel.eta(1))^2);
%a0_360 = rad2deg(wrapTo2Pi(deg2rad(a0)));
% 
% 
% phi_TS = atan2((vessel.eta(2)-tracks.eta(2)), (vessel.eta(1) - tracks.eta(1)));
% psi_TSR = tracks.eta(3) - vessel.eta(3) - phi_TS;
% 
% phi_TS = wrapTo2Pi(phi_TS);
% psi_TSR = wrapTo2Pi(psi_TSR);

% 1 = HO
% 2 = GW
% 3 = SO
% 4 = OT
% 5 = SF
if cflag == 0 && dist < 150 %% TODO: MAKE PROPER FUNCTION TO CHECK WHEN IT'S TIME TO CHECK COLREGs FLAG
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