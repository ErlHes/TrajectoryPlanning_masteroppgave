function [reference_trajectory_los,end_of_path_index] = reference_trajectory_from_dynamic_los_guidance(OS, parameters, h, N)
% N = N;   %number of samples (-)
dt = h;    %sampling time (s)
T = N * dt;

end_of_path_index = N;
end_of_path_is_set = false;
%if(mod(parameters.ocp.run_period,dt) >0)
%    disp('Warning: ocp dt and run_period does not match up');
%end

subtraction_from_ocp_period = 0;

reference_trajectory_los = zeros(4,N+1-subtraction_from_ocp_period);
reference_trajectory_los(:,1) = [OS.eta(1:2,1);...
                                 OS.eta_dot(1:2,1)];

% course_profile_los = zeros(1,N+1-subtraction_from_ocp_period);


OS.eta_ref = OS.eta;
OS.eta_dot_ref = OS.eta_dot;

dt_refinement = round((T/N)/0.1);
parameters.guidance.dt = (T/N)/dt_refinement;

for i=1:ceil((N-subtraction_from_ocp_period)*dt_refinement)
    if(i/dt_refinement==(N+1-subtraction_from_ocp_period))
        break;
    end
    [OS, end_of_path] = guidance_simulator_multiAgent(OS,parameters.guidance);

    if(end_of_path && ~end_of_path_is_set)
        end_of_path_index = round(i/(dt_refinement))+1;
        end_of_path_is_set = true;
    end
    OS.eta = OS.eta_ref;
    OS.eta_dot = OS.eta_dot_ref;

    if(mod(i,dt_refinement) == 0) && (i>0)
        reference_trajectory_los(:,i/(dt_refinement)+1) = [OS.eta(1:2,1);...
                                                           OS.eta_dot(1:2,1)];
%         course_profile_los(1,i/(dt_refinement)+1) = OS.eta(3,1);
    end
end

end
