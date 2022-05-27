function Tau = pickthree(tau)
Fx = tau(1);
Fy = tau(2);
Fn = tau(3);
Fxn = tau(4);
Fyn = tau(5);
Fnn = tau(6);

Tau = [Fx,Fy,Fn]';

if(Fx < abs(Fxn))
    Tau(1) = Fxn;
end
if(Fy < abs(Fyn))
    Tau(2) = Fyn;
end
if(Fn < abs(Fnn))
    Tau(3) = Fnn;
end

end