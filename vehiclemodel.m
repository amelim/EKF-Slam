function xv = vehiclemodel(xv, V, W, dt)
    xv = [xv(1) + dt*V*cos(xv(3) + W); 
        xv(2) + dt*V*sin(xv(3) + W); 
        pilimit(xv(3) + dt*V*sin(W)/0.1);];
end
