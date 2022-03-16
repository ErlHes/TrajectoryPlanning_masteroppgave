function [N, h] = DynamicHorizon(vessel, tracks)
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
Timetogoal = distancetogoal / vessel.nu(1);

%Getting past relevant TS:
%some function
%return TimetopassTS

%compare time to pass goal and time to pass TS, we want to keep the
%smallest of theese two

%max time of n minutes:
maxminutes = 3; 
maxseconds = maxminutes * 60;
finaltime = min([Timetogoal, maxseconds]);

h = 0.65; % statisk for nå.
N = ceil(finaltime / h);

end