transit_OS = [
    -70,-50;...
    50,0]';

OS_velocity = [
    2.0;...
    2.0;...
    2.0]';

agents = [
get_agent(100, [-70,-50,0]', [0,0,0]',[5,3], 2, 3, OS_velocity, transit_OS,1)];
