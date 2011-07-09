%Vehicle Control params
V = 0.3;
MAXW = 120.0*pi/180.0;
DT_CONTROLS = 0.1;

WHEEL_BASE = 0.1;

%Control Noise
sigmaV = 0.03;
sigmaW = 1.0*pi/180.0;
Q = [sigmaV^2 0; 0 sigmaW^2];

%Observation params
MAX_RANGE = 1.9;

%observation noise
sigmaR = 0.05;
sigmaB = (1.0*pi/180);
R = [sigmaR^2 0; 0 sigmaB^2];

%min distance to be considered at waypoin
AT_WAYPOINT = 0.03;
