function tagPosition = weightedLeastSquares(anchorRanges, rssiRatios)
    % Function to estimate the tag position using Weighted Least Squares (WLS)
    % Input:
    %   anchorRanges - Nx1 vector of distances (ranges) from each anchor to the tag
    %   rssiRatios - Nx1 vector of fp_rssi / rx_rssi ratios for each anchor
    % Output:
    %   tagPosition - estimated [x, y] position of the tag

    anchorPositions = [0, 0; 0.485, 11.19; 5.478, 11.254; 6.866, 0]; 

    % Distances for each anchor
    r0 = anchorRanges(1);
    r1 = anchorRanges(2);
    r2 = anchorRanges(3);
    r3 = anchorRanges(4);

    % Use rssiRatios as weights for each anchor
    W = diag(rssiRatios);

    % Reference anchor
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

    % Solve for the tag position using Weighted Least Squares
    A_T = transpose(A);
    alpha_inv = inv(A_T * W * A);
    beta = A_T * W * b;
    tagPosition = alpha_inv * beta;
end
