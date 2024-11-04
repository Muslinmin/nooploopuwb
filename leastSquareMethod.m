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

    % Distances for each anchor
    r0 = anchorRanges(1);
    r1 = anchorRanges(2);
    r2 = anchorRanges(3);
    r3 = anchorRanges(4);

    % Reference anchor and other anchors
    A0 = anchorPositions(1, :);
    A1 = anchorPositions(2, :);
    A2 = anchorPositions(3, :);
    A3 = anchorPositions(4, :);

    % Set up the system of equations in the form A*x = b
    A = [
        2 * (A1(1) - A0(1)), 2 * (A1(2) - A0(2));
        2 * (A2(1) - A0(1)), 2 * (A2(2) - A0(2));
        2 * (A3(1) - A0(1)), 2 * (A3(2) - A0(2))
    ];

    b = [
        r0^2 - r1^2 - A0(1)^2 + A1(1)^2 - A0(2)^2 + A1(2)^2;
        r0^2 - r2^2 - A0(1)^2 + A2(1)^2 - A0(2)^2 + A2(2)^2;
        r0^2 - r3^2 - A0(1)^2 + A3(1)^2 - A0(2)^2 + A3(2)^2
    ];

    % Solve for the tag position using Least Squares (no weights)
    tagPosition = (A' * A) \ (A' * b);
end
