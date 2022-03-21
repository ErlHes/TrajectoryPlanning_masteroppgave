function [pos_OS, vel_OS] = VesselWPReadout(vessel,i)
%Reads out the position and velocity of the vessel at each waypoint in it's
%transit. If the index of the wpt we're reading is the same as current wpt
%we instead read out current position and velocity.
    pos_OS = vessel.wp(1:2,i);
    Heading_OS = atan2(vessel.wp(2,i+1)-vessel.wp(2,i),vessel.wp(1,i+1)-vessel.wp(1,i));
    vel_OS = rotZ(Heading_OS)*vessel.nu;
    vel_OS = vel_OS(1:2);
    if i == vessel.current_wp %instead of start of current wp, use current location.
        pos_OS = vessel.eta(1:2);
        vel_OS = vessel.eta_dot(1:2);
    end
    
end