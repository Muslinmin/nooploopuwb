%% Map and Path Planning Setup
% Load and prepare the map
mapImage = imread('layout.jpg');
grayImage = rgb2gray(mapImage);
binaryImage = imbinarize(grayImage);
binaryImage = ~binaryImage;

% Create the binary occupancy map and inflate obstacles for safety
resolution = 20;
occupancyMap = binaryOccupancyMap(binaryImage, resolution);
inflationRadius = 0.2;  % Adjust as needed
inflate(occupancyMap, inflationRadius);

% Define goal position
goalPos = [25, 13];
startPos = [5,5];
goalGrid = world2grid(occupancyMap, goalPos);

% Plot map, anchors, and goal
show(occupancyMap);
hold on;
plot(goalPos(1), goalPos(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

%anchorCoordinates = [0, 0; 0.175, 11.22; 9.388, 11.26; 9.308, 0];
%for i = 1:size(anchorCoordinates, 1)
%    plot(anchorCoordinates(i, 1), anchorCoordinates(i, 2), 'bo', 'MarkerSize', 8);
%end

% Set up the initial car position and appearance
[carImage, ~, carAlpha] = imread('car.png');
desiredWidth = 2;
desiredHeight = desiredWidth / (size(carImage, 2) / size(carImage, 1));
carImagePlot = image('CData', carImage, 'XData', [0, desiredWidth], 'YData', [0, desiredHeight]);
set(carImagePlot, 'AlphaData', carAlpha);

tagPositionPlot = plot(NaN, NaN, 'rx', 'MarkerSize', 10, 'LineWidth', 2);


if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

s = serial('COM5', 'BaudRate', 115200);
fopen(s);


pathPlanningInterval = 2; % Time in seconds between path planning updates
lastPlanningTime = tic;

while true
    % Fetch or update the latest tag data
    [tagCoordinates, ~, yaw] = ExtractAnchorAndTagInfo(s);
    startPos = tagCoordinates;

% Rotate the car image and its alpha channel based on the yaw value
rotatedCarImage = imrotate(carImage, -yaw, 'bilinear', 'crop');
rotatedCarAlpha = imrotate(carAlpha, -yaw, 'bilinear', 'crop');  % Rotate the alpha channel as well

% Calculate new position to center the rotated image at startPos
carXData = [startPos(1) - desiredWidth / 2, startPos(1) + desiredWidth / 2];
carYData = [startPos(2) - desiredHeight / 2, startPos(2) + desiredHeight / 2];

% Update the car image position, orientation, and transparency
set(carImagePlot, 'CData', rotatedCarImage, ...
                  'AlphaData', rotatedCarAlpha, ...  % Apply rotated alpha data for transparency
                  'XData', carXData, ...
                  'YData', carYData);

    % Update the tag position on the plot
    set(tagPositionPlot, 'XData', startPos(1), 'YData', startPos(2));

    % Check if enough time has passed to update the path
    if toc(lastPlanningTime) > pathPlanningInterval
        % Reinitialize the planner with inflated map
        inflatedMap = copy(occupancyMap);
        inflate(inflatedMap, inflationRadius);
        
        % Convert start position to grid indices
        startGrid = world2grid(inflatedMap, startPos);
        
        % Plan the path from the current position to the goal
        planner = plannerAStarGrid(inflatedMap);
        try
            [path, ~] = plan(planner, startGrid, goalGrid);
            
            % Convert path to world coordinates for visualization
            pathWorld = grid2world(occupancyMap, path);

            % Remove previous path plot if any
            if exist('pathPlotHandle', 'var') && isvalid(pathPlotHandle)
                delete(pathPlotHandle);
            end
            pathPlotHandle = plot(pathWorld(:,1), pathWorld(:,2), 'r', 'LineWidth', 2);

            disp('Path planning updated');
        catch
            disp('Path planning failed - possibly due to start location near obstacle');
        end

        % Reset the path planning timer
        lastPlanningTime = tic;
    end

    % Pause for continuous updates
    pause(0.01);
end

% Close serial port when done
fclose(s);
