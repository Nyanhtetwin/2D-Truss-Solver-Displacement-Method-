function KK = Bar2D2Node_Assembly(KK_global, k_elem, i, j)
% Bar2D2Node_Assembly Assembles the 2D element stiffness matrix into the global stiffness matrix.
%   KK = Bar2D2Node_Assembly(KK_global, k_elem, i, j)
%   Inputs:
%       KK_global: Current global stiffness matrix (or initialized zero matrix)
%       k_elem: 4x4 element stiffness matrix in global coordinates
%       i: Global node number for the first node of the element
%       j: Global node number for the second node of the element
%   Output:
%       KK: Updated global stiffness matrix

% Degrees of freedom (DOFs) mapping for nodes i and j in a 2D system
DOF_map = [
    2*i - 1; % DOF for x-displacement at node i
    2*i;     % DOF for y-displacement at node i
    2*j - 1; % DOF for x-displacement at node j
    2*j      % DOF for y-displacement at node j
];

% Initialize KK with the global matrix passed in
KK = KK_global;

% Loop through the 4x4 element stiffness matrix and add its contributions
% to the correct positions in the global stiffness matrix
for n1 = 1:4
    global_dof1 = DOF_map(n1);
    for n2 = 1:4
        global_dof2 = DOF_map(n2);
        KK(global_dof1, global_dof2) = KK(global_dof1, global_dof2) + k_elem(n1, n2);
    end
end

end
