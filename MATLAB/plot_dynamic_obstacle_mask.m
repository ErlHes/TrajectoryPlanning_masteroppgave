function [handle] = plot_dynamic_obstacle_mask(static_obs,fig)
if isempty(static_obs)
handle = 1/fig*0;    
else
handle = 1/fig*0;
[~,col] = find(isnan(static_obs(1,:)));

for i=1:(size(col,2)-1)
   element = static_obs(:,(col(i)+1):(col(i+1)-1));
%     plot([element(2,end), element(2,:)], [element(1,end), element(1,:)], 'b');
   handle = fill([element(2,end), element(2,:)], [element(1,end), element(1,:)],get_rgb('green_feature'));
   set(handle, 'facealpha', 0.5)
end
end
end

