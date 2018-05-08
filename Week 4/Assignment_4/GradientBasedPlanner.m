function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
% Normalize gx and gy
nrm = sqrt(gx.^2 + gy.^2);
gx = gx ./ nrm;
gy = gy ./ nrm;
route = [start_coords];
dist = norm(end_coords - start_coords);
its = 1;
current = start_coords;

while (its <= max_its && dist >= 2)
    
    i = round(current(1));
    j = round(current(2));
    
    gx_current = gx(j,i);
    gy_current = gy(j,i);
    
    current_pos = [(current(1) + gx_current) (current(2) + gy_current)];
    
    route = [route; current_pos];
   
    dist = norm(end_coords - current_pos);
    
    its = its + 1;
    
    current = current_pos;
end

route = double(route);

% *******************************************************************


end
