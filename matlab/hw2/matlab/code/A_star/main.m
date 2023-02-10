% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;

set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 10 - ceil(2*rand);
yTarget = 10 - ceil(2*rand);
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
visualize_map(map, 0);
% Waypoint Generator Using the A* 
path = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path);
