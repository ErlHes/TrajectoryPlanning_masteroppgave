function S_range= CPE_safety_from_range(range,params)
%CPE_SAFETY_FROM_RANCE calculates a safety score based on the distance
%between two vessels. 
%   Detailed explanation goes here

ranges = params.ranges;
scores = params.range_scores;

if range <= ranges(1) 
   S_range = scores(1);
   return;
elseif range >= ranges(end)
   S_range = scores(end); 
   return;
end

i = sum(ranges <= range);
reminder = range - ranges(i);
S_range = scores(i) + reminder/(ranges(i+1)-ranges(i))*(scores(i+1)-scores(i));


end

