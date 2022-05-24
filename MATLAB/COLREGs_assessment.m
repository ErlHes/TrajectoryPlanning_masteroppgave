function [flag, dCPA, tCPA] = COLREGs_assessment(vessel,tracks, cflag)
%% THIS FUNCTION EVALUATES ONE TARGET SHIP ONLY. TO EVALUATE MORE THE FUNCTION MUST BE CALLED FOR EACH TARGET SHIP IN YOUR SITUATION.
% a13 = 112.5; % Overtaking tolerance
% a14 = rad2deg(pi/8); % head-on tolerance
% a15 = rad2deg(pi/8); % crossing aspect limit

%% Calculate dCPA and tCPA, check if COLREGs assessment is needed:
% [dCPA, tCPA] = ClosestApproach(vessel.eta(1:2), tracks.eta(1:2), vessel.eta_dot(1:2), tracks.eta_dot(1:2));
[dCPAlist, tCPAlist,pos_OS_list, pos_TS_list] = getCPAlist(vessel,tracks);

%Keep the lowest dCPA found, this is the only dCPA we're interested in
%If there should ever be multiple equally low dCPAs we are in a unsupported
%special case that needs more development.
dCPA = min(dCPAlist);
dCPAminlist = find(dCPAlist == dCPA);
tCPA = tCPAlist(dCPAminlist(1));
pos_OS = pos_OS_list(1:3,dCPAminlist);
pos_TS = pos_TS_list(1:3,dCPAminlist);

[TSdCPAlist, TStCPAlist, ts_pos_TS_list, ts_pos_OS_list] = getCPAlist(tracks,vessel);
TSdCPA = min(TSdCPAlist);
TSdCPAminlist = find(TSdCPAlist == TSdCPA);

%HACKJOB
%This is a failsafe to prevent MATLAB from throwing an error and halting
%the program should any of the Target Ships in the simulation be at their
%final destination.
if(~isempty(TStCPAlist))
    TStCPA = TStCPAlist(TSdCPAminlist(1));
    tspos_OS = ts_pos_OS_list(1:3,TSdCPAminlist);
    tspos_TS = ts_pos_TS_list(1:3,TSdCPAminlist);
else
    TStCPA = 0;
    tspos_OS = [0 0 0]';
    tspos_TS = [100 100]';
end
%END of HACKJOB

if TSdCPA < dCPA
    dCPA = TSdCPA;
    tCPA = TStCPA;
    pos_OS = tspos_OS;
    pos_TS = tspos_TS;
end

%Nå vet vi hva dCPA og tCPA er, kan nå sammenligne med en eller annen
%kvantitet for å se om det er høvelig å sette COLREGs flag på TS.
%HVIS vi ønsker å sette COLREGs flag må vi også vite hvor OS og TS er i
%forhold til hverandre, og hvilke kurs begge har når vi starter på banen
%som tar oss til denne dCPAen.
OSareal = vessel.size(1)*vessel.size(2);
TSareal = tracks.size(1)*tracks.size(2);

dCPAgrense = (OSareal + TSareal + max(OSareal,TSareal)) / 2; % En eller annen funksjon av størrelser
%Hvis problemet blir unfeasible kan det hende vi blir nødt til å senke
%denne grensen, men det er en funksjon for en annen dag.

tCPAgrense = 3 * dCPAgrense;



%% Conduct COLREGs assessment
if (dCPA < dCPAgrense) && (tCPA < tCPAgrense) && cflag == 0
    % Angles between OS and TS
    phi_1 = rad2deg(pi/8);
%     phi_1 = rad2deg(pi/15);
    phi_2 = 112.5;
    
    b0 = rad2deg(wrapTo2Pi(atan2((pos_TS(2)-pos_OS(2)),(pos_TS(1)-pos_OS(1))) - ssa(pos_OS(3)))); % Relative from OS to TS

    b0_180 = ssa(b0, 'deg');
    
    a0 = rad2deg(ssa(atan2(pos_OS(2)-pos_TS(2),pos_OS(1)-pos_TS(1)) - pos_TS(3))); % Relative from TS to OS
    
    % dist = sqrt((tracks.eta(2) - vessel.eta(2))^2 + (tracks.eta(1) - vessel.eta(1))^2);
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
else
    flag = cflag;
end

if dCPA > (dCPAgrense+30)
    flag = 0;
end


end