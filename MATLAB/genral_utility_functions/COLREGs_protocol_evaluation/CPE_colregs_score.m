function S = CPE_colregs_score( alpha,beta,classification, params)
% Classification value
% 1: SF - Safe
% 2: SO - Stand-on
% 3: OTs - Overtaking on starboard side
% 4: OTp - Overtaking on port side
% 5: HO - Head-on
% 6: GW - Give-way.


switch classification
    case 1 % Safe
        S = 1;
    case 2 % Stand on
        S = CPE_give_way_crossing_score(beta, alpha);
    case 3 % Overtaking on starboard side
        S = 1;
    case 4 % Overtaking on port side
        S = 1;
    case 5
        S = CPE_head_on_score(alpha,beta, params);
    case 6
        S = CPE_give_way_crossing_score(alpha, beta);
    otherwise
        S = 1;
        disp('Warning: Case is "otherwise" in switch case in "CPE_colregs_score"');
end


end