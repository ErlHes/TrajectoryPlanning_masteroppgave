run('/home/emilht/MATLAB/genral_utility_functions/COLREGs_protocol_evaluation/colregs_protocol_parameters.m');



%% Test 1
eta = [80,0,0]';
eta_c = [60,0,pi/2]';
u = 1;
uc = 1;
[cpa, tcpa, beta, alpha] = get_cpa(eta,u, eta_c, uc, CPE_parameters.safety)