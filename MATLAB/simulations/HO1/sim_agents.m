%% OWN SHIP
transit_os = [
    50, 125;...
    225, 125]';
Heading_os = atan2(transit_os(2,2)-transit_os(2,1),transit_os(1,2)-transit_os(1,1));

velocity_os = [
    2.0;...
    2.0]';

%% TARGET SHIP 1
transit_ts1 = [
    225, 130;...
    50, 125;...
    ]';
Heading_ts1 = atan2(transit_ts1(2,2)-transit_ts1(2,1),transit_ts1(1,2)-transit_ts1(1,1));

velocity_ts1 = [
    2.0;...
    2.0]';

%% Gather agents
agents = [
   get_agent(101,[transit_ts1(1,1), transit_ts1(2,1),Heading_ts1]',[0,0,0]',[5,3],2,2,velocity_ts1,transit_ts1,1),...
   get_agent(100,[transit_os(1,1), transit_os(2,1), Heading_os]',[velocity_os(1),0,0]',[5,3],2,3,velocity_os, transit_os,1)];
