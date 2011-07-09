function [Gt, Gu] = calcJacobian(muLast, v, w, deltaT)
    angle = muLast(3);
    %xUp = -(v/w)*sin(angle) + (v/w)*sin(angle + w*deltaT);
    xUp = deltaT*v*cos(angle + w);
    %yUp = (v/w)*cos(angle) - (v/w)*cos(angle + w*deltaT);
    yUp = deltaT*v*sin(angle + w);
    Gt = [1,0,xUp; 0,1,yUp; 0,0,1];
    Gu = [deltaT*cos(angle + w) xUp;
          deltaT*sin(angle + w) yUp;
          deltaT*sin(w)/0.1 v*deltaT*cos(w)/0.1;];
endfunction

