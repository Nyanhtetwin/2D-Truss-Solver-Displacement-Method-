function k = Bar2D2Node_Stiffness_fromCoords(E, A, x1, y1, x2, y2)
% Bar2D2Node_Stiffness_fromCoords Calculates the 4x4 stiffness matrix for a 2D bar (truss) element.
% This function automatically calculates the element's angle from its coordinates
%
%   k = Bar2D2Node_Stiffness_fromCoords(E, A, x1, y1, x2, y2)
%   Inputs:
%       E: Elastic modulus (Young's Modulus)
%       A: Cross-sectional area
%       x1, y1: Global coordinates of the first node
%       x2, y2: Global coordinates of the second node
%   Output:
%       k: 4x4 element stiffness matrix in global coordinates

% Calculate the length of the element
L = sqrt((x2 - x1)^2 + (y2 - y1)^2);

% Determine the angle of the element with the global X-axis
delta_x = x2 - x1;
delta_y = y2 - y1;
alpha_rad = atan2(delta_y, delta_x);

% Calculate cosine and sine of the angle
C = cos(alpha_rad);
S = sin(alpha_rad);

% Define C-squared, S-squared, and CS terms for convenience
C2 = C^2;
S2 = S^2;
CS = C * S;

% Calculate the coefficient EA/L
coefficient = E * A / L;

% Construct the 4x4 element stiffness matrix in global coordinates
k = coefficient * [
    C2,  CS, -C2, -CS;
    CS,  S2, -CS, -S2;
    -C2, -CS,  C2,  CS;
    -CS, -S2,  CS,  S2
];

end
