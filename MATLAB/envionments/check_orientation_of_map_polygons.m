% Check that all polygons are given in clockwise direction

figure(888)
clf(888)
hold on;
grid on;

[~,col] = find(isnan(static_obs(1,:)));

   tic

for i=1:(size(col,2)-1)
   element = static_obs(:,(col(i)+1):(col(i+1)-1));
   tf = ispolycw(element(2,:),element(1,:));
   if(tf)
       fill([element(2,end), element(2,:)], [element(1,end), element(1,:)],get_rgb('light_grey'));
   end
end
   t = toc
   disp(toc);
