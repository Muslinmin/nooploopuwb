%% Binary Occupancy
% Load Your Map Image
mapImage = imread('layout.jpg');

% Convert the Image to Grayscale
grayImage = rgb2gray(mapImage);

% Transform the grayscale image into a binary image
binaryImage = imbinarize(grayImage);

% Invert binary image if 0 is free space and 1 is occupied space
binaryImage = ~binaryImage;

% Define resolution (cells per meter)
resolution = 20;

% Create the Binary Occupancy Map
occupancyMap = binaryOccupancyMap(binaryImage, resolution);
inflateRadius = 0.1;
inflate(occupancyMap,inflateRadius);
show(occupancyMap);
% Define start and goal positions in world coordinates [x, y]
startPos = [20, 11]; % Replace with actual coordinates
goalPos = [36, 8];    % Replace with actual coordinates

% Convert start and goal positions to grid indices
startGrid = world2grid(occupancyMap, startPos);
goalGrid = world2grid(occupancyMap, goalPos);

% Initialize the A* planner
planner = plannerAStarGrid(occupancyMap);

% Plan the path
[path, pathInfo] = plan(planner, startGrid, goalGrid);

% Convert path from grid indices to world coordinates
pathWorld = grid2world(occupancyMap, path);

% Visualize the occupancy map
show(occupancyMap);
hold on;

% Plot the planned path
plot(pathWorld(:,1), pathWorld(:,2), 'r', 'LineWidth', 2);

% Mark start and goal positions
plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPos(1), goalPos(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

hold off;
