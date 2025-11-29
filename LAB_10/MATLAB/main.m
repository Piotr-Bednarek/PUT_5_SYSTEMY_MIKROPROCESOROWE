%% Matrix multiplication unit test
%%% File info 
%
% ************************************************************************
%
%  @file     unit_test_mat_mult.m
%  @author   Adrian Wojcik
%  @version  2.0
%  @date     15-Dec-2020 08:55:55
%  @brief    Generates source, header and data (.csv) files for matrix
%            multiplication unit test
%
% ************************************************************************
%
close all; clc;
clear A B C_REF
%% MATRIX MULTIPLICATION

% square matrix size
n = 4; % [-]

A = single(rand(n,n));
B = single(rand(n,1));
C_REF = A*B;

disp("LHS ARGUMENT (MATRIX)");
disp(A);

disp("RHS ARGUMENT (COLUMN VECTOR)");
disp(B);

disp("REFERENCE RESULT:");
disp(C_REF);

%% EXPORT MATRICES TO .C/.H FILES
generate_mat('A', A);
generate_mat('B', B);
generate_mat('C', zeros(size(C_REF)));
generate_mat('C_REF', C_REF);

%% MOVE .C/.H AND .CSV FILES TO Components CATALOG
srcDest = '../Core/Src';
cFiles = dir('*.c');
csvFiles = dir('*.csv');
for i = 1:length(cFiles)
    movefile(cFiles(i).name, fullfile(srcDest, cFiles(i).name));
end
for i = 1:length(csvFiles)
    movefile(csvFiles(i).name, fullfile(srcDest, csvFiles(i).name));
end

incDest = '../Core/Inc';
hFiles = dir('*.h');
for i = 1:length(hFiles)
    movefile(hFiles(i).name, fullfile(incDest, hFiles(i).name));
end