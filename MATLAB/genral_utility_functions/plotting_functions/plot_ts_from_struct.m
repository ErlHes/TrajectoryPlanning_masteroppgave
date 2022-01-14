function handle = plot_ts_from_struct(vessel,color1, scale)
trc = zeros(2,6);

psi=vessel.eta(3);
l=vessel.size(1)*scale+2;
b=vessel.size(2)*scale;


p = vessel.eta(1:2,1);
trc(:,1) = p + l/2*rot2(psi); 
trc(:,2) = p + l/4*rot2(psi) + b/2*rot2(psi+pi/2);
trc(:,3) = p + l/2*rot2(psi+pi) + b/2*rot2(psi+pi/2);
trc(:,4) = p + l/2*rot2(psi+pi) - b/2*rot2(psi+pi/2);
trc(:,5) = p + l/4*rot2(psi) - b/2*rot2(psi+pi/2);
trc(:,6) = trc(:,1);


% handle = plot(trc(2,:) , trc(1,:),'color',option,'linewidth',2 );
handle1 = fill(trc(2,:) , trc(1,:),color1 );
% handle2 = plot(trc(2,:), trc(1,:), 'color',color1, 'linewidth',2);
handle = [handle1];
end

function mt = rot2(psi)
mt = [cos(psi); sin(psi)];
end