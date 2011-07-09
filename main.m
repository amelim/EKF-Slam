landmarks = [1 2 2 3 2; 1 4 3 2 1; 1 2 3 4 5];
waypoints = [0 2 2 3 4 4 0 0; 0 0 2 3 4 7 7 0;];
data = ekfslam(landmarks, waypoints);
