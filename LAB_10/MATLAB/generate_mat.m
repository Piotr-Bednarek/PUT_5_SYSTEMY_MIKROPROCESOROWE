function generate_mat(name, x)
% GENERATE_MAT Generates source file (.c), header file (.h) and text data 
% file (.csv) for ARM CMSIS DSP matrix instance using float32_t.

HEADER_STR = {
   ['/* MATLAB GENERATED HEADER FILE: ' name '_mat.h */']
   ['#ifndef INC_' upper(name) '_MAT_H_']
   ['#define INC_' upper(name) '_MAT_H_']
    ''
    '#include "arm_math.h"'
    ''
   ['#define ' upper(name) '_ROWS ' num2str(size(x,1))]
   ['#define ' upper(name) '_COLS ' num2str(size(x,2))]
   ['extern float32_t ' upper(name) '_DATA[' upper(name) '_ROWS*' upper(name) '_COLS];']
   ['extern arm_matrix_instance_f32 ' name ';']
    ''
   ['#endif /* INC_' upper(name) '_MAT_H_ */']
};

fileID = fopen([name '_mat.h'],'w');
for i = 1:length(HEADER_STR)
    fprintf(fileID,'%s\n', HEADER_STR{i});
end
fclose(fileID);

% Zamieniamy macierz na wektor wierszami
x_vec = x';
x_vec = x_vec(:);

% Tworzymy plik CSV
vec2csvfile([name '.csv'], x_vec);

% Generowanie pliku .c
SOURCE_STR = {
   ['/* MATLAB GENERATED SOURCE FILE: ' name '_mat.c */']
   ['#include "' name '_mat.h"']
   ['float32_t ' upper(name) '_DATA[' upper(name) '_ROWS*' upper(name) '_COLS] = ']
    '{'
};

% Dodanie danych z CSV w formie float32
for i = 1:length(x_vec)
    if i < length(x_vec)
        SOURCE_STR{end+1} = ['  ' num2str(x_vec(i),'%.8ff') ','];
    else
        SOURCE_STR{end+1} = ['  ' num2str(x_vec(i),'%.8ff')];
    end
end
SOURCE_STR{end+1} = '};';
SOURCE_STR{end+1} = ['arm_matrix_instance_f32 ' name ';'];

fileID = fopen([name '_mat.c'],'w');
for i = 1:length(SOURCE_STR)
    fprintf(fileID,'%s\n', SOURCE_STR{i});
end
fclose(fileID);

end
