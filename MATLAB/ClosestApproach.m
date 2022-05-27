function [dCPA, tCPA] = ClosestApproach(pos_OS, pos_TS, vel_OS, vel_TS)
%Returns the distance at closest point of approach and time until closest
%point of approach. Assuming both vessels maintain a fixed course and
%speed.
vel_AB = vel_OS - vel_TS;
pos_BA = pos_TS - pos_OS;

tCPA = 0;
if (norm(vel_AB,2) > 0)
    tCPA = dot(pos_BA,vel_AB) / norm(vel_AB,2)^2;
end

dCPAfunc = (pos_OS + tCPA * vel_OS) - (pos_TS + tCPA * vel_TS);
dCPA = norm(dCPAfunc,2);

if tCPA < 0
    dCPA = norm(pos_BA,2);
    tCPA = 0;
end

end