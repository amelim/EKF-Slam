function [W,iwp]=computerotation(wp, iwp, minD, W, maxW, dt)
    global vtrue
    cwp = wp(:,iwp);
    d = sqrt((cwp(1) - vtrue(1))^2 + (cwp(2) - vtrue(2))^2);
    if d < minD
        iwp += 1;
        if iwp > size(wp,2)
            iwp=1;
        end
        cwp = wp(:,iwp);
    end

    %Angle between the two points
    W = pilimit(atan2(cwp(2) - vtrue(2), cwp(1)-vtrue(1)) - vtrue(3));
    if abs(maxW*dt) < W
        W = sign(W)*maxW*dt;
    end
    
end

