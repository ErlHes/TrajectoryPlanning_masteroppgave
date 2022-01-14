function [handle] = plot_static_obs(static_obs,fig)
handle = 0; % Not in used for now
if isempty(static_obs)

else

figure(fig)
hold on;
    
% Find index of all nan-values (which are used to separate each polygon).
[~,col] = find(isnan(static_obs(1,:)));
for i=1:(size(col,2)-1)
   element = static_obs(:,(col(i)+1):(col(i+1)-1));
   fill([element(2,end), element(2,:)], [element(1,end), element(1,:)],get_rgb('land'));
end
end
end

