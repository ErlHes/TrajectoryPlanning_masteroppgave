island1 = [
    230, 55;...
    235, 75;...
    220, 100;...
    210, 75;...
    230, 55]';

island2 = [
    150, 75;...
    160, 97;...
    146, 86;...
    150, 75]';

island3 = [
    100, 90;...
    136, 110;...
    142, 140;...
    78, 138;...
    87, 120;...
    100, 90]';

separator = [nan;nan];
static_obs = [separator, island1,...
              separator, island2,...
              separator, island3, separator];
    