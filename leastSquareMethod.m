function tagPosition = leastSquareMethod(anchorRanges)
    % Function to estimate the tag position using Least Squares (no weights)
    % Input:
    %   anchorRanges - Nx1 vector of distances (ranges) from each anchor to the tag
    % Output:
    %   tagPosition - estimated [x, y] position of the tag

    % Fixed anchor positions
    anchorPositions = [
        0, 0;           % A0 (Reference anchor)
       -0.055, 2.062;   % A1
        2.006, 2.421;   % A2
        2.086, 0        % A3
    ];

    % Set up the system of equations in the form A*x = b
    % Use the first anchor as the reference point
    A = 2 * (anchorPositions(2:end, :) - anchorPositions(1, :));

    % Construct vector b using squared distances and coordinates
    b = (anchorRanges(2:end).^2 - anchorRanges(1)^2 + sum(anchorPositions(1, :).^2, 2)) - ...
        (sum(anchorPositions(2:end, :).^2, 2));

    % Solve for the tag position using Least Squares (without weights)
    tagPosition = (A' * A) \ (A' * b);
end
