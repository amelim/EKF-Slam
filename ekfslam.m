function data = ekfslam(lm, wp)
    format compact;
    configfile;
    %setup animation
    screensize = get(0, 'ScreenSize')*0.75;
    fig = figure('Position', [0 0 screensize(3), screensize(4)]);
    plot(lm(1,:),lm(2,:),'b*')
    hold on, axis equal, grid on
    MAXX = max([max(lm(1,:)) max(wp(1,:))]);
    MAXY= max([max(lm(2,:)) max(wp(2,:))]);
    MINX = min([min(lm(1,:)) min(wp(1,:))]);
    MINY= min([min(lm(2,:)) min(wp(2,:))]);

    axis([MINX-1 MAXX+1 MINY-1 MAXY+1])
    h = setup_animations;
    veh = [0 -WHEEL_BASE -WHEEL_BASE; 0 -2 -2];

    %initialize states
    global vtrue XX PX DATA
    vtrue = zeros(3,1);
    XX = zeros(3,1);
    DR = zeros(3,1);
    PX = zeros(3);

    dt = DT_CONTROLS;
    dtsum = 0;
    iwp = 1;
    W = 0;
    counter = 0;
    path_count = 0;
    associations = []; 

    sim_time = 0;

    DATA = init_storage(XX, PX, vtrue, DR);

    %mainloop
    while iwp ~= 0 
        sim_time += dt;
        counter++;
        [W, iwp] = computerotation(wp, iwp, AT_WAYPOINT, W, MAXW, dt);

        vtrue = vehiclemodel(vtrue, V, W, dt);
        %Control noise
        Vn = V + randn(1)*sqrt(Q(1,1));
        Wn = W + randn(1)*sqrt(Q(2,2));

        %Predict our estimate with our jacobians
        [Gt, Gu] = calcJacobian(XX, Vn, Wn, dt); 
        PX(1:3, 1:3) = Gt*PX(1:3, 1:3)*Gt.' + Gu*Q*Gu.';
        if length(PX) > 3
            PX(1:3,4:end) = Gt*PX(1:3,4:end);
            PX(4:end,1:3) = PX(1:3,4:end)';
        end
            

        XX(1:3,:) = vehiclemodel(XX, Vn, Wn, dt);
        DR = vehiclemodel(DR, Vn, Wn, dt);


        %Get our observations
        %z = observed lm, zindex = it's id and relation to the assoc. table
        z = []; zindex = [];
        znew = true;
        for i=lm
            dist = sqrt((i(1) - vtrue(1))^2 + (i(2) - vtrue(2))^2);
            if dist <= MAX_RANGE
                %All landmarks which can be observed described by range,bearing,id
                bearing = atan2((i(2) - vtrue(2)),(i(1) - vtrue(1)));

                l = [dist; pilimit(bearing - vtrue(3)); i(3)];
                z = [z l]
                %Is this a new landmark? If not, get it's index in the map
                for y = 1:length(associations)
                    if i(3) == associations(y)
                        zindex = [zindex y];
                        znew = false;
                    end
                end
                %Never seen this lm before, add it's ID to the association table
                if znew
                    associations = [associations i(3)];
                    %Increase the size of our mean and covar
                    muJ = [XX(1) + l(1)*cos(l(2) + XX(3)); XX(2) + l(1)*sin(l(2) + XX(3))];
                    XX = [XX; muJ(1); muJ(2)];
                    covars = length(PX);
                    PX(covars + 1, covars + 1) = 1; 
                    PX(covars + 2, covars + 2) = 1;
                end
            end
            znew = true;
        end
        
        %observation jacobian
        j = 0;
        for zi = 1:size(z,2)
            %Calculated with current waypoint zloc
            zloc = z(:,zi);
            for r = 1:length(associations) 
                if zloc(3) == associations(r) 
                    j = r;
                end
            end
            if (j==0)
                disp('Cannot find landmark');
            end
             
            jind = j*2 + 2;
            [zhat, H] = observemodel(XX, zloc, jind); 

            PHt = PX*H.';
            S = H*PHt + R;
            SInv = inv(S);
            %SInv = (SInv + SInv')*0.5;
            %PSD_check = chol(SInv);
            K = PHt*SInv;
            diff = [zloc(1) - zhat(1); zloc(2) - zhat(2)];
            %K*[zloc(1) - zhat(1); zloc(2) - zhat(2)]
            XLast = XX(1:3);

            XX = XX + K*[zloc(1) - zhat(1); zloc(2) - zhat(2)];

            L = K*S*K';
            PX = (eye(length(PX)) - K*H)*PX;
            %PX = PX - ((L + L')*0.5)
            


            j=0;
        end
        

        store_data(XX, PX, vtrue, DR);
        %calc covar ellipse
        vehEl = make_veh_ellipse(XX,PX);
        set(h.vehStat, 'string',  num2str(XX(3)*180/pi));
        try
            set(h.cova, 'xdata', vehEl(1,:), 'ydata', vehEl(2,:))
            set(h.xv, 'xdata', XX(1,:), 'ydata', XX(2,:))
            set(h.dr, 'xdata', DR(1,:), 'ydata', DR(2,:))
            set(h.pthtrue, 'xdata', DATA.truth(1,1:DATA.i), 'ydata', DATA.truth(2,1:DATA.i))
            set(h.pthdead, 'xdata', DATA.dead(1,1:DATA.i), 'ydata', DATA.dead(2,1:DATA.i))
            set(h.pth, 'xdata', DATA.path(1,1:DATA.i), 'ydata', DATA.path(2,1:DATA.i))
            set(h.xf, 'xdata', XX(4:2:end), 'ydata', XX(5:2:end))
            set(h.xt, 'xdata', vtrue(1,:), 'ydata', vtrue(2,:))
            drawnow
        catch
            disp('whoops')
        end
    end
        %animations
end

function h = setup_animations()
    h.xv = plot(0,0,'ro','erasemode','xor');
    h.xt = plot(0,0,'bo','erasemode','xor');
    h.dr = plot(0,0,'go','erasemode','xor');
    h.pth = plot(0,0,'r','markersize',2,'erasemode','background');
    h.pthtrue = plot(0,0,'b','markersize',2,'erasemode','background');
    h.pthdead = plot(0,0,'g','markersize',2,'erasemode','background');
    h.obs = plot(0,0,'k','erasemode', 'xor');
    %h.timeelapsed = annotation('textbox', [0.89 0.9 0.1 0.05]);
    h.xf = plot(0,0,'r+');
    h.cova = plot(0,0,'r','erasemode', 'xor');
    h.vehStat = text(0, 8, 'ERROR');
end

function p= make_veh_ellipse(x, P)
    N = 10;
    inc = 2*pi/N;
    phi = 0:inc:2*pi;
    circ = 2*[cos(phi); sin(phi)];

    r = sqrtm_2by2(P(1:2,1:2));
    a = r*circ;
    p(2,:) = [a(2,:)+x(2) NaN];
    p(1,:) = [a(1,:)+x(1) NaN];
end

function data = init_storage(x, P, vtrue, DR)
    data.i=1;
    data.path = x;
    data.truth = vtrue;
    data.dead = DR;
    data.state(1).x = x;
    data.state(1).P = diag(P);
end

function store_data(x, P, vtrue, DR)
    global DATA
    CHUNK = 500;
    len = size(DATA.path,2);
    if (DATA.i == len)
        disp('increase')
        if len < CHUNK, len= CHUNK; end
        DATA.path = [DATA.path zeros(3,len)];
        DATA.truth = [DATA.truth zeros(3,len)];
        DATA.dead = [DATA.dead zeros(3,len)];
        pack
    end
    i = DATA.i + 1;
    DATA.i = i;
    DATA.path(:,i) = x(1:3);
    DATA.truth(:,i) = vtrue;
    DATA.dead(:,i) = DR;
    DATA.state(i).x = x;
    DATA.state(i).P = diag(P);
end
