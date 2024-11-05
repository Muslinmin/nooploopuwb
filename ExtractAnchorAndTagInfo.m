function [tagCoordinates, anchorData, pitchRoll] = ExtractAnchorAndTagInfo(s)
    % Initialize outputs
    tagCoordinates = [0, 0];
    anchorData = zeros(4, 3); % Array for up to 4 anchors, each with distance, fpRssi, rxRssi
    pitchRoll = [0, 0]; % Array for pitch and roll values

    % Read a line of data from the serial port
    rawData = fscanf(s, '%s'); % Read as string

    % Split data into segments by semicolons
    segments = strsplit(rawData, ';');

    % Parse the tag's coordinates
    if ~isempty(segments{1})
        tagCoords = strsplit(segments{1}, ',');
        if numel(tagCoords) == 2
            % Remove scaling for tag coordinates
            tagCoordinates = [str2double(tagCoords{1}), str2double(tagCoords{2})];
        end
    end

    % Parse the anchor data
    for i = 2:min(length(segments)-1, 5) % Anchor data starts from the 2nd segment and ends before pitch/roll
        anchorVals = strsplit(segments{i}, ',');
        if numel(anchorVals) == 3
            % Convert each value back to its original scale (divide by 10)
            anchorData(i-1, :) = [str2double(anchorVals{1}) / 10, ...
                                  str2double(anchorVals{2}) / 10, ...
                                  str2double(anchorVals{3}) / 10];
        end
    end

    % Parse pitch and roll data from the last segment
    if length(segments) >= 6
        pitchRollVals = strsplit(segments{end-1}, ',');
        if numel(pitchRollVals) == 2
            pitchRoll = [str2double(pitchRollVals{1}) / 10, str2double(pitchRollVals{2}) / 10];
        end
    end

    % Close the serial port (optional)
    % fclose(s);
end
