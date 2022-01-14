function [M,C,D] = comp_matrices_surge_decoupled(nu)
%COMP_MATRICES Summary of this function goes here
%   Detailed explanation goes here
u   = nu(1);
v   = nu(2);
r   = nu(3);

%% Parameters for coupled milliAmpere model
mm11 = [2389.657020]; % Measured weight is m = 1667kg
mm12 = [0];
mm13 = [0];
mm21 = [0];
mm22 = [2533.911]; 
mm23 = [62.386];
mm31 = [0];
mm32 = [28.141];
mm33 = [5068.910];

Xu      = [-27.632]; 
Xuu     = [-110.064];
Xuuu    = [-13.965];

Xv      = [0]; %13

Xr      = [0];

Yu      = [0]; %15

Yv      = [-52.947882419472997]; %16
Yvv     = [-116.4866337294660];
Yvvv    = [-24.313479009650798];
Yrv     = [-1540.383915627830];

Yr      = [24.732147729183499];
Yvr     = [572.1413450046700];
Yrr     = [-115.457991449463];

Nu      = [0];

Nv      = [3.524988764908190]; %24
Nvv     = [-0.832914416785907];
Nrv     = [336.8276949935520];

Nr      = [-122.8609796220180]; %27
Nrr     = [-874.4282494384980]; %28
Nrrr    = [0.000]; %29
Nvr     = [-121.957738563599]; %30

%% Mass matrix. M + MA

m11 = mm11; m12 = mm12; m13 = mm13;
m21 = mm21; m22 = mm22; m23 = mm23;
m31 = mm31; m32 = mm32; m33 = mm33;

M = [m11 m12 m13; m21 m22 m23; m31 m32 m33];
% M = skew(M);
% inv_M = M^-1;

%% Coreolis matrix. C + CA.

ca11 = 0; ca12 = 0; ca13 = m21*u - m22*v - m23*r;
ca21 = 0; ca22 = 0; ca23 = m11*u + m12*v + m13*r;
ca31 = -ca13; ca32 = -ca23; ca33 = 0;

CA = [ca11 ca12 ca13; ca21 ca22 ca23; ca31 ca32 ca33];
C = CA; 

%% Damping matrix. D.
DL = [-Xu -Xv -Xr; -Yu -Yv -Yr; -Nu -Nv -Nr];

d11 = -Xuu*abs(u) - Xuuu*u^2; d12 = 0;                                   d13 = 0;
d21 = 0;                      d22 = -Yvv*abs(v) - Yrv*abs(r) - Yvvv*v^2; d23 = -Yvr*abs(v) - Yrr*abs(r);
d31 = 0;                      d32 = -Nvv*abs(v) - Nrv*abs(r);            d33 = -Nvr*abs(v) - Nrr*abs(r) - Nrrr*r^2 ;
DNL = [d11 d12 d13; d21 d22 d23; d31 d32 d33];

D = DL + DNL;
end
