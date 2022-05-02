%% OWN SHIP
transit_os = [
    550, 200;...
    640, 400;...
    680, 460
    700, 600;...
    750, 770;...
    1000, 880;
    1100, 1000;...
    1150, 1200;...
    ]';
Heading_os = atan2(transit_os(2,2)-transit_os(2,1),transit_os(1,2)-transit_os(1,1));

velocity_os = [
    2.0;...
    2.0;...
    2.0;...
    2.0;...
    2.0;...
    2.0]';

%% TARGET SHIP 1
transit_ts1 = [
    1600, 320;...
    1200, 500;...
    900, 600;...
    650, 800;...
    530, 1200;...
    ]';
Heading_ts1 = atan2(transit_ts1(2,2)-transit_ts1(2,1),transit_ts1(1,2)-transit_ts1(1,1));

velocity_ts1 = [
    7.0;...
    7.0;...
    7.0;...
    7.0;...
    7.0]';

%% TARGET SHIP 2
transit_ts2 = [
    600, 1200;...
    660, 900;...
    800, 750;...
    780, 450;...
    600, 130;...
    ]';
Heading_ts2 = atan2(transit_ts2(2,2)-transit_ts2(2,1),transit_ts2(1,2)-transit_ts2(1,1));

velocity_ts2 = [
    5.0;...
    5.0;...
    5.0;...
    5.0;...
    5.0]';

%% TARGET SHIP 3
transit_ts3 = [
    1400, 1550;...
    1300, 1400;...
    1250, 1200;...
    1200, 1000;...
    1130, 900;...
    790, 800;...
    650, 400;...
    ]';
Heading_ts3 = atan2(transit_ts3(2,2)-transit_ts3(2,1),transit_ts3(1,2)-transit_ts3(1,1));

velocity_ts3 = [
    4.0;...
    4.0;...
    3.0;...
    3.0;...
    3.0]';

%% TARGET SHIP 4
transit_ts4 = [
    600, 335;...
    700, 300;...
    703, 302;...
    705, 300;...
    590, 335;...
    600, 330;...
    605, 335;...
    710, 300;...
    710, 305;...
    705, 300;...
    600, 335;...
    600, 330;...
    605, 335;...
    710, 300;...
    710, 305;...
    705, 300;...
    600, 335;...
    ]';
Heading_ts4 = atan2(transit_ts4(2,2)-transit_ts4(2,1),transit_ts4(1,2)-transit_ts4(1,1));

velocity_ts4 = [
    2.0;...
    1.0;...
    1.0;...
    1.0;...
    2.0;...
    1.0;...
    1.0;...
    1.0;...
    2.0;...
    1.0;...
    1.0;...
    1.0;...
    2.0;...
    1.0;...
    1.0;...
    1.0;...
    ]';

%% TARGET SHIP 5
transit_ts5 = [
    1050, 1050;...
    1260, 1020;...
    1261, 1023;...
    1262, 1025;...
    1260, 1025;...
    1050, 1050;...
    1051, 1053;...
    1052, 1055;...
    1050, 1055;...
    1260, 1020;...
    ]';
Heading_ts5 = atan2(transit_ts5(2,2)-transit_ts5(2,1),transit_ts5(1,2)-transit_ts5(1,1));

velocity_ts5 = [
    4.0;...
    4.0;...
    4.0;...
    4.0;...
    4.0;...
    4.0;...
    4.0;...
    4.0]';

%% TARGET SHIP 6
transit_ts6 = [
    515, 1155;...
    530, 1120;...
    660, 1040;...
    680, 940;...
    700, 900;...
    1300, 750;...
    1550, 650;...
    1650, 625;...
    ]';
Heading_ts6 = atan2(transit_ts6(2,2)-transit_ts6(2,1),transit_ts6(1,2)-transit_ts6(1,1));

velocity_ts6 = [
    2.0;...
    3.0;...
    6.0;...
    4.0;...
    7.0;...
    7.0]';

%% TARGET SHIP 7
transit_ts7 = [
    850, 700;...
    880, 700;...
    880, 730;...
    850, 730;...
    850, 700;...
    880, 700;...
    880, 730;...
    880, 700;...
    ]';

Heading_ts7 = atan2(transit_ts7(2,2)-transit_ts7(2,1),transit_ts7(1,2)-transit_ts7(1,1));

velocity_ts7 = [
    1.0;...
    1.0;...
    1.0;...
    1.0;...
    1.0;...
    1.0;...
    ]';

%% Gather agents
agents = [
   get_agent(101,[transit_ts1(1,1), transit_ts1(2,1),Heading_ts1]',[transit_ts1(1),0,0]',[5,3],2,2,velocity_ts1,transit_ts1,1),...
   get_agent(102,[transit_ts2(1,1), transit_ts2(2,1),Heading_ts2]',[transit_ts2(1),0,0]',[5,3],2,2,velocity_ts2,transit_ts2,1),...
   get_agent(103,[transit_ts3(1,1), transit_ts3(2,1),Heading_ts3]',[transit_ts3(1),0,0]',[5,3],2,2,velocity_ts3,transit_ts3,1),...
   get_agent(104,[transit_ts4(1,1), transit_ts4(2,1),Heading_ts4]',[transit_ts4(1),0,0]',[5,3],2,2,velocity_ts4,transit_ts4,1),...
   get_agent(105,[transit_ts5(1,1), transit_ts5(2,1),Heading_ts5]',[transit_ts5(1),0,0]',[5,3],2,2,velocity_ts5,transit_ts5,1),...
   get_agent(106,[transit_ts6(1,1), transit_ts6(2,1),Heading_ts6]',[transit_ts6(1),0,0]',[5,3],2,2,velocity_ts6,transit_ts6,1),...
   get_agent(107,[transit_ts7(1,1), transit_ts7(2,1),Heading_ts7]',[transit_ts7(1),0,0]',[5,3],2,2,velocity_ts7,transit_ts7,1),...
   get_agent(100,[transit_os(1,1), transit_os(2,1), Heading_os]',[velocity_os(1),0,0]',[5,3],2,3,velocity_os, transit_os,1)];
