%% OWN SHIP
transit_os = [
    480, 150;...
    350, 68;...
    25, 300;...
    -20, 270;...    
    ]';
Heading_os = wrapTo2Pi(atan2(transit_os(2,2)-transit_os(2,1),transit_os(1,2)-transit_os(1,1)));

velocity_os = [
    2.0;...
    2.0;...
    2.0;...
    2.0]';

%% TARGET SHIP 1
transit_ts1 = [
    227, 155;...
    350, 68;...
    530, 200;...
    ]';
Heading_ts1 = atan2(transit_ts1(2,2)-transit_ts1(2,1),transit_ts1(1,2)-transit_ts1(1,1));

velocity_ts1 = [
    2.0;...
    2.0;...
    2.0]';

%% TARGET SHIP 2
transit_ts2 = [
    1040, 400;...
    350, 450;...
    200, 400;...
    25, 300;...
    -20, 270;...
    ]';
Heading_ts2 = atan2(transit_ts2(2,2)-transit_ts2(2,1),transit_ts2(1,2)-transit_ts2(1,1));

velocity_ts2 = [
    4.0;...
    4.0;...
    4.0;...
    4.0]';

%% TARGET SHIP 3
transit_ts3 = [
    530, 220;...
    350, 68;...
    25, 300;...
    -20, 270;...
    ]';

Heading_ts3 = wrapTo2Pi(atan2(transit_ts3(2,2)-transit_ts3(2,1),transit_ts3(1,2)-transit_ts3(1,1)));

velocity_ts3 = [
    2.0;...
    3.0;...
    3.0;...
    3.0]';

%% TARGET SHIP 4
transit_ts4 = [
    -20, 270;...   
    25, 300;...
    350, 68;...
    480, 150;...
    ]';

Heading_ts4 = wrapTo2Pi(atan2(transit_ts4(2,2)-transit_ts4(2,1),transit_ts4(1,2)-transit_ts4(1,1)));

velocity_ts4 = [
    2.0;...
    2.0;...
    2.0;...
    2.0]';


%% Gather agents
agents = [
   get_agent(101,[transit_ts1(1,1), transit_ts1(2,1),Heading_ts1]',[velocity_ts1(1),0,0]',[5,3],2,2,velocity_ts1,transit_ts1,1),...
   get_agent(102,[transit_ts2(1,1), transit_ts2(2,1),Heading_ts2]',[velocity_ts2(1),0,0]',[5,3],2,2,velocity_ts2,transit_ts2,1),...
   get_agent(103,[transit_ts3(1,1), transit_ts3(2,1),Heading_ts3]',[velocity_ts3(1),0,0]',[5,3],2,2,velocity_ts3,transit_ts3,1),...
   get_agent(104,[transit_ts4(1,1), transit_ts4(2,1),Heading_ts4]',[velocity_ts4(1),0,0]',[5,3],2,2,velocity_ts4,transit_ts4,1),...
   get_agent(100,[transit_os(1,1), transit_os(2,1), Heading_os]',[velocity_os(1),0,0]',[5,3],2,3,velocity_os, transit_os,1)];
