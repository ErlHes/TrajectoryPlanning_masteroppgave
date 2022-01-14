function tracking_data = get_tracking_data_struct(id, eta, vel, size)
tracking_data = struct;
tracking_data.id = id;
tracking_data.eta = eta;
tracking_data.vel = vel;
tracking_data.size = size;
end
