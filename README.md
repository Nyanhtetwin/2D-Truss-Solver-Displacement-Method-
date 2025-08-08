2D Truss Finite Element Analysis
This repository contains a set of MATLAB scripts for performing a simple 2D truss analysis using the Finite Element Method (FEM). The main script, SolveTruss.m, orchestrates the entire analysis, from defining the problem to calculating displacements, stresses, and reaction forces.

Getting Started
To run the analysis, simply place all the .m files in the same directory and execute the SolveTruss.m script in MATLAB. The script will output the results directly to the Command Window.

File Descriptions
SolveTruss.m:
The main script that defines the truss geometry, material properties, and applied loads. It calls the other functions to build the global stiffness matrix, solve for nodal displacements, and post-process the results.

Bar2D2Node_Stiffness_fromCoords.m:
A function that calculates the 4x4 element stiffness matrix for a 2D bar element. It automatically determines the element's angle from its node coordinates.

Bar2D2Node_Assembly.m:
A function responsible for assembling a single element's stiffness matrix into the larger global stiffness matrix of the entire structure.

Bar2D2Node_CalculateAll.m:
A helper function that, for a single element, calculates its stiffness matrix, internal nodal forces, and axial stress in one convenient call.
