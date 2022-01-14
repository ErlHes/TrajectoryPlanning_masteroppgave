function handle = plot_os(eta, option, scale)
trc = zeros(2,7);
l = 3*scale;   %half length of ferry
b = 2.5*scale; %half width of ferry
n = eta(1);
e = eta(2);
p = eta(3);
trc(:,1) = [n+l*cos(p), e+l*sin(p)];
trc(:,2) = [n+b*cos(p+pi/6), e+b*sin(p+pi/6)];
trc(:,3) = [n+b*cos(p+5*pi/6), e+b*sin(p+5*pi/6)];
trc(:,4) = [n-l*cos(p), e-l*sin(p)];
trc(:,5) = [n+b*cos(p-5*pi/6), e+b*sin(p-5*pi/6)];
trc(:,6) = [n+b*cos(p-pi/6), e+b*sin(p-pi/6)];
trc(:,7) = [n+l*cos(p), e+l*sin(p)];
% handle = plot(trc(2,:), trc(1,:), 'LineWidth' , 2 ,'Color', option);
handle = fill(trc(2,:) , trc(1,:),option );
end