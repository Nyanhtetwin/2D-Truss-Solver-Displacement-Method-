% This script demonstrates how to solve a simple 2D truss problem
% using defined functions.

% Clear workspace and command window
clc;
clear;

%% 1. Preprocessing: Define Geometry and Material Properties
%---------------------------------------------------------------
% Define material properties
E = 2.1e11; % Young's Modulus (in N/m^2)
A = 0.001;  % Cross-sectional area (in m^2)

% Define nodal coordinates [x, y]
node_coords = [
    0, 0;    % Node 1 (pin support)
    2, 0;    % Node 2 (pin support)
    1, 1.5;  % Node 3 (load application point)
];

% Define element connectivity [node_i, node_j]
elements = [
    1, 3; % Element 1 connects Node 1 and Node 3
    2, 3; % Element 2 connects Node 2 and Node 3
];

% Number of nodes and elements
num_nodes = size(node_coords, 1);
num_elements = size(elements, 1);

% Total degrees of freedom (DOFs)
num_dofs = 2 * num_nodes;

% Initialize global stiffness matrix and force vector
KK = zeros(num_dofs, num_dofs);
F = zeros(num_dofs, 1);

% Define external force (a vertical force of -1000 N at Node 3)
force_node = 3;
force_y = -1000; % Downward force
F(2 * force_node) = force_y;


%% 2. Element-wise Calculation and Assembly
%---------------------------------------------------------------
% Loop through each element to calculate its stiffness matrix
% and assemble it into the global stiffness matrix.
for e = 1:num_elements
    % Get node indices for the current element
    node_i = elements(e, 1);
    node_j = elements(e, 2);

    % Get the coordinates of the element's nodes
    x1 = node_coords(node_i, 1);
    y1 = node_coords(node_i, 2);
    x2 = node_coords(node_j, 1);
    y2 = node_coords(node_j, 2);

    fprintf('Assembling Element %d (nodes %d and %d)\n', e, node_i, node_j);

    % Call the separate function to calculate the element stiffness matrix
    k_elem = Bar2D2Node_Stiffness_fromCoords(E, A, x1, y1, x2, y2);
    
    % Call the separate function to assemble the element into the global matrix
    KK = Bar2D2Node_Assembly(KK, k_elem, node_i, node_j);
end

% Display the assembled global stiffness matrix
fprintf('\nGlobal Stiffness Matrix (KK):\n');
disp(KK);


%% 3. Apply Boundary Conditions and Solve
%---------------------------------------------------------------
% Identify constrained degrees of freedom (DOFs)
% Node 1 (0,0) is fixed: u1_x = 0, u1_y = 0. DOFs 1 and 2.
% Node 2 (2,0) is also now fixed: u2_x = 0, u2_y = 0. DOFs 3 and 4.
fixed_dofs = [1, 2, 3, 4];
unknown_dofs = setdiff(1:num_dofs, fixed_dofs);

% Reduce the global stiffness matrix and force vector
K_mod = KK(unknown_dofs, unknown_dofs);
F_mod = F(unknown_dofs);

% Solve for the unknown displacements
U_mod = K_mod \ F_mod;

% Construct the full global displacement vector U
U = zeros(num_dofs, 1);
U(unknown_dofs) = U_mod;

% Display the global displacement vector
fprintf('Global Displacement Vector (U):\n');
fprintf(' u1_x = %e\n u1_y = %e\n u2_x = %e\n u2_y = %e\n u3_x = %e\n u3_y = %e\n', U(1), U(2), U(3), U(4), U(5), U(6));


%% 4. Post-processing: Calculate Forces and Stresses
%---------------------------------------------------------------
% Loop through each element to calculate internal forces and stresses
fprintf('\nElement Results:\n');
for e = 1:num_elements
    % Get node indices and coordinates for the current element
    node_i = elements(e, 1);
    node_j = elements(e, 2);
    x1 = node_coords(node_i, 1);
    y1 = node_coords(node_i, 2);
    x2 = node_coords(node_j, 1);
    y2 = node_coords(node_j, 2);

    % Extract the element's displacement vector from the global vector
    u_elem = [U(2*node_i - 1); U(2*node_i); U(2*node_j - 1); U(2*node_j)];
    
    % Call the separate function to calculate all results at once
    [~, forces, stress] = Bar2D2Node_CalculateAll(E, A, x1, y1, x2, y2, u_elem);
    
    fprintf('--------------------------------------------------\n');
    fprintf('Element %d (Nodes %d -> %d):\n', e, node_i, node_j);
    fprintf(' Axial Stress (N/m^2): %e\n', stress);
    fprintf(' Internal Forces (N):\n');
    fprintf('  F_ix = %f\n', forces(1));
    fprintf('  F_iy = %f\n', forces(2));
    fprintf('  F_jx = %f\n', forces(3));
    fprintf('  F_jy = %f\n', forces(4));
    fprintf('--------------------------------------------------\n');
end

%% 5. Calculate Global Reaction Forces
%---------------------------------------------------------------
% The reaction forces F_R are calculated from the partitioned global
% stiffness matrix and the solved displacements U_mod.
% The formula is F_R = K_fu * U_mod
% where K_fu is the submatrix corresponding to fixed DOFs and unknown DOFs.

% Extract the partitioned stiffness matrix K_fu
K_fu = KK(fixed_dofs, unknown_dofs);

% Calculate the reaction forces
F_reactions = K_fu * U_mod;

fprintf('\nGlobal Reaction Forces (N):\n');
fprintf(' F at Node 1, x-dir (DOF 1) = %f\n', F_reactions(1));
fprintf(' F at Node 1, y-dir (DOF 2) = %f\n', F_reactions(2));
fprintf(' F at Node 2, x-dir (DOF 3) = %f\n', F_reactions(3));
fprintf(' F at Node 2, y-dir (DOF 4) = %f\n', F_reactions(4));
