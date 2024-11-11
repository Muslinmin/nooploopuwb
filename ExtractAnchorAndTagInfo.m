function [tagCoordinates, anchorRanges, rssiRatios, yaw] = ExtractAnchorAndTagInfo(s)
    % Initialize outputs
    tagCoordinates = [0, 0];
    anchorRanges = zeros(4, 1); % Array for up to 4 anchors, each with distance
    rssiRatios = zeros(3, 1);   % Array for RSSI ratios for anchors 2, 3, and 4
    yaw = 0; % Initialize yaw as a single value

    % Read a line of data from the serial port
    rawData = fscanf(s, '%s'); % Read as string

    % Split data into segments by semicolons
    segments = strsplit(rawData, ';');

    % Parse the tag's coordinates
    if ~isempty(segments{1})
        tagCoords = strsplit(segments{1}, ',');
        if numel(tagCoords) == 2
            % Convert tag coordinates
            tagCoordinates = [str2double(tagCoords{1}), str2double(tagCoords{2})];
        end
    end

    % Parse the anchor data
    anchorCount = min(length(segments)-2, 4); % Ensure we process up to 4 anchors
    for i = 1:anchorCount
        anchorVals = strsplit(segments{i+1}, ',');
        if numel(anchorVals) == 3
            % Convert each value
            distance = str2double(anchorVals{1});
            fpRssi = str2double(anchorVals{2});
            rxRssi = str2double(anchorVals{3});

            % Store distance in anchorRanges
            anchorRanges(i) = distance;

            % Calculate RSSI ratio for anchors 2, 3, and 4 (exclude the first anchor)
            if i > 1
                rssiRatios(i-1) = fpRssi / rxRssi;
            end
        end
    end

    % Parse the yaw data from the last segment
    if length(segments) >= 6
        yaw = str2double(segments{end}); % Parse yaw as a single value
    end

    % Close the serial port (optional)
    % fclose(s);
end
