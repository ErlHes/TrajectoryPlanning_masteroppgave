
yaml_struct = ReadYaml('/home/emilht/NTNU-aFerry/workspace/src/ros_cbf_based_colav_utils/config/cbf_based_colav_utils.yaml');

lidar_settings = yaml_struct.lidar_settings;
% 
% lidar_settings.num_sectors = 12;
% lidar_settings.max_range = 50;
% lidar_settings.min_range = 2;
% 
% lidar_settings.rotation_to_body = -deg2rad(34);
% 
% lidar_settings.filter_jitter_on_ranges = 1;
% lidar_settings.num_consecutive = 3;
% lidar_settings.tolerance = 3;
% lidar_settings.remove_ranges_on_tracks = 1;
% lidar_settings.tracks_mask_radius = 6; 
% 

lidar_settings_bus = lidar_settings;
simulink_running = 1;
if(simulink_running)
Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = lidar_settings_bus;
Simulink_obj.CoderInfo.StorageClass = 'ExportedGlobal';
lidar_settings_bus = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(lidar_settings_bus.Value);
LidarSettings = eval(busInfo.busName);
lidar_settings_bus.DataType='Bus: Object_Info_Interface';
clear(busInfo.busName);
clear busInfo;
end