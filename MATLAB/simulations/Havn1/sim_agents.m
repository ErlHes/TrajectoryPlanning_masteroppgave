%% OWN SHIP
transit_os = [
    30, 210;...
    80, 208;...
    95, 207;...
    120, 150;...
    120, 30;...
    ]';
Heading_os = atan2(transit_os(2,2)-transit_os(2,1),transit_os(1,2)-transit_os(1,1));

velocity_os = [
    2.0;...
    2.0;...
    2.0;...
    2.0;...
    2.0]';

%% TARGET SHIP 1
transit_ts1 = [
    93, 30;...
    110, 150;...
    125, 200;...
    250, 210;...
    ]';
Heading_ts1 = atan2(transit_ts1(2,2)-transit_ts1(2,1),transit_ts1(1,2)-transit_ts1(1,1));

velocity_ts1 = [
    2.0;...
    2.0;...
    2.0;...
    2.0]';

%% TARGET SHIP 2
transit_ts2 = [...
    150, 115;...
    133, 115;...
    126, 150;...
    110, 170;...
    0, 170;...
    ]';

Heading_ts2 = atan2(transit_ts2(2,2)-transit_ts2(2,1),transit_ts2(1,2)-transit_ts2(1,1));

velocity_ts2 = [...
    1.0;...
    1.0;...
    1.0;...
    1.0;...
    1.0;...
    ]';
%% Gather agents
agents = [
   get_agent(101,[transit_ts1(1,1), transit_ts1(2,1),Heading_ts1]',[velocity_ts1(1),0,0]',[5,3],2,2,velocity_ts1,transit_ts1,1),...
   get_agent(102,[transit_ts2(1,1), transit_ts2(2,1),Heading_ts2]',[velocity_ts2(1),0,0]',[5,3],2,2,velocity_ts2,transit_ts2,1),...
   get_agent(100,[transit_os(1,1), transit_os(2,1), Heading_os]',[velocity_os(1),0,0]',[5,3],2,3,velocity_os, transit_os,1)];
