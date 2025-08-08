function [k_elem, forces, stress] = Bar2D2Node_CalculateAll(E, A, x1, y1, x2, y2, u_elem)
% Bar2D2Node_CalculateAll Calculates the stiffness matrix, nodal forces, and axial stress
% for a single 2D bar element in one function call.
%
%   [k_elem, forces, stress] = Bar2D2Node_CalculateAll(E, A, x1, y1, x2, y2, u_elem)
%   Inputs:
%       E: Elastic modulus (Young's Modulus)
%       A: Cross-sectional area
%       x1, y1: Global coordinates of the first node
%       x2, y2: Global coordinates of the second node
%       u_elem: 4x1 vector of element nodal displacements in global coordinates
%   Outputs:
%       k_elem: 4x4 element stiffness matrix in global coordinates
%       forces: 4x1 vector of element nodal forces [F_ix; F_iy; F_jx; F_jy]
%       stress: Axial stress in the element

% First, calculate the element stiffness matrix using the simplified function
k_elem = Bar2D2Node_Stiffness_fromCoords(E, A, x1, y1, x2, y2);

% Next, calculate the nodal forces using the stiffness matrix and displacements
% This uses the standard finite element formula: forces = k * u
forces = k_elem * u_elem;

% Finally, calculate the axial stress
% Note: This calculation is a bit redundant with the forces calculation,
% but it's kept separate to follow the logic of the original Bar2D2Node_Stress.m
% Calculate the length of the element
L = sqrt((x2 - x1)^2 + (y2 - y1)^2);

% Determine the angle of the element with the global X-axis
delta_x = x2 - x1;
delta_y = y2 - y1;
alpha_rad = atan2(delta_y, delta_x);

% Calculate cosine and sine of the angle
C = cos(alpha_rad);
S = sin(alpha_rad);

% Calculate stress using the formula: stress = E/L * [-C -S C S] * u
stress_vector_multiplier = [-C, -S, C, S];
stress = (E / L) * (stress_vector_multiplier * u_elem);

end
