%% Create Bus for Objects
%*****************************%


%% Define the "object" data structure for the object detection interface.
% This is the structure on the bus-interface between the object detection
% and the deliberate COLAV system.
obj = struct;
obj.eta    = [0 0 0];
obj.vel    = 0;
obj.size   = [0,0];
obj.time   = 0;
obj.active = 0;

object_info_interface = [obj,obj,obj,obj,obj,obj,obj,obj,obj,obj];



% Object Detection Interface.
Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = object_info_interface;
Simulink_obj.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
object_info_interface = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(object_info_interface.Value);
Object_Info_Interface = eval(busInfo.busName);
object_info_interface.DataType='Bus: Object_Info_Interface';
clear(busInfo.busName);
clear busInfo;

%Situation Parameters.
Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = object_detection;
Simulink_obj.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
object_detection = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(object_detection.Value);
Object_Detection = eval(busInfo.busName);
object_detection.DataType='Bus: Object_Detection';
clear(busInfo.busName);
clear busInfo;