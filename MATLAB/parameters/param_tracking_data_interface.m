

yaml_struct = ReadYaml('/home/emilht/NTNU-aFerry/workspace/src/ros_cbf_based_colav_utils/config/cbf_based_colav_utils.yaml');
% run('piren_frame_dynamic_obstacle_mask.m');

parameters_tracking_data_interface = yaml_struct.parameters_tracking_data_interface;
% parameters_tracking_data_interface.dynamic_obstacle_mask = dynamic_obstacle_mask;


param_tracking_data_interface_bus = parameters_tracking_data_interface;
simulink_running = 1;
if(simulink_running)
Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = param_tracking_data_interface_bus;
Simulink_obj.CoderInfo.StorageClass = 'ExportedGlobal';
param_tracking_data_interface_bus = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(param_tracking_data_interface_bus.Value);
ParamTrackingDataInterface = eval(busInfo.busName);
param_tracking_data_interface_bus.DataType='Bus: ParamTrackingDataInterface';
clear(busInfo.busName);
clear busInfo;
end