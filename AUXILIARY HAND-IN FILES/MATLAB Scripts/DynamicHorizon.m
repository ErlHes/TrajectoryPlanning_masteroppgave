function [N, h] = DynamicHorizon(vessel, dynamic_obs)
% Calculate an appropriate number of time steps and step length based on
% distance to goal and other vessels.

%Distance to goal:
%dist = sqrt((tracks.eta(2) - vessel.eta(2))^2 + (tracks.eta(1) - vessel.eta(1))^2);
distancetogoal = 0;
for i = size(vessel.wp,2):-1:vessel.current_wp+2
    distbetweenWP = sqrt((vessel.wp(1,i) - vessel.wp(1,i-1))^2 + ((vessel.wp(2,i) - vessel.wp(2,i-1))^2));
    distancetogoal = distancetogoal + distbetweenWP;
end
distancetonextWP = sqrt((vessel.wp(1,vessel.current_wp+1) - vessel.eta(1))^2 + ((vessel.wp(2,vessel.current_wp+1) - vessel.eta(2))^2));
distancetogoal = distancetogoal + distancetonextWP;
if vessel.nu(1) < 0.001
    vessel.nu(1) = 0.001;
end
Timetogoal = distancetogoal / vessel.nu(1);


%Getting past relevant TS:
%some function
%return TimetopassTS
if(~isempty(dynamic_obs))
    allTcpas = [dynamic_obs.tcpa];
    maxtCPA = max(allTcpas) + 20; % Add time, we want to pass the encounter, not just reach it.
end

%compare time to pass goal and time to pass TS, we want to keep the
%smallest of theese two

%max time of n minutes:
maxminutes = 5; 
maxseconds = maxminutes * 60;
minminutes = 3;
minseconds = minminutes * 60;

% WRONG
% minstetid = max(minseconds, maxtCPA);
% finaltime = min([Timetogoal, maxseconds, minstetid]);

% CORRECT, but never used
if (false) % <- TODO: check if any cflags are set.
    maxtime = min([Timetogoal, maxseconds]);
    finaltime = min([maxtime, maxtCPA]); 
else
    finaltime = min([Timetogoal, maxseconds]);
end
% finaltime = min([Timetogoal, maxseconds]);




h = 0.5; % statisk for nå.
N = ceil(finaltime / h);

end