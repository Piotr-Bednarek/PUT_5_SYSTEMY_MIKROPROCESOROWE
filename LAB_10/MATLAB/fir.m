%% FIR FILTER DESIGN AND TEST SCRIPT

%% 0. Ścieżki do katalogów projektu
srcDest = '../Core/Src';
incDest = '../Core/Inc';

if ~exist(srcDest, 'dir'); mkdir(srcDest); end
if ~exist(incDest, 'dir'); mkdir(incDest); end

%% 1. Parametry filtra FIR
Fs = 1;           % Częstotliwość próbkowania normalizowana
Fp = 0.1;         % Graniczna częstotliwość pasa przepustowego (0..0.5)
Fs_stop = 0.15;   % Graniczna częstotliwość pasa zaporowego
Ap = 1;           % Tłumienie w paśmie przepustowym [dB]
As = 60;          % Tłumienie w paśmie zaporowym [dB]

% Estymacja rzędu filtra metodą Harrisa
delta_f = Fs_stop - Fp;             
N = ceil((Fs * As)/(22*delta_f));
N = max(N, 10);                      
disp(['Rząd filtra FIR: ', num2str(N)]);

%% 2. Generowanie współczynników filtra FIR
b = fir1(N, Fp, 'low'); 
writematrix(b', 'FIR_COEFFS.csv');

%% 3. Generowanie sygnału testowego
n = 0:999;                   
x = sin(2*pi*0.02*n) + 0.5*randn(size(n));
writematrix(x', 'TEST_SIGNAL.csv');

%% 4. Referencyjna filtracja w MATLAB
y_ref = filter(b, 1, x);
writematrix(y_ref', 'REF_SIGNAL.csv');

%% 5. Wizualizacja filtra
[H,f] = freqz(b,1,1024,Fs);
figure;
plot(f,20*log10(abs(H)));
grid on;
xlabel('Częstotliwość [Hz]');
ylabel('Amplituda [dB]');
title('Charakterystyka częstotliwościowa filtra FIR');

%% Odczyt danych z CSV STM32 w Core/Src
filename = fullfile(srcDest, 'wynik.csv');  
if exist(filename, 'file')
    data = readmatrix(filename);  
    data = data(~isnan(data));    
else
    error('Plik wynik.csv nie istnieje w katalogu Core/Src');
end

% Wykres STM32
figure;
plot(data, '-o');
xlabel('Próbka n');
ylabel('Wartość y\_test');
title('Wynik filtra FIR z STM32');
grid on;

%% Obliczenie RMSE STM32 vs MATLAB
% Dopasowanie długości wektorów
% data_cmp = data(1:minLen);
% y_ref_cmp = y_ref(1:minLen);

data_stm = data(:);      % Zamienia na pionowy (N x 1)
y_ref_col = y_ref(:);    % Zamienia na pionowy (N x 1)  

minLen = min(length(data_stm), length(y_ref_col));
stm_aligned = data_stm(1:minLen);
ref_aligned = y_ref_col(1:minLen);

rmse_stm = sqrt(mean((stm_aligned - ref_aligned).^2));

% Wyświetlenie
disp(['RMSE STM32 vs MATLAB = ', num2str(rmse_stm)]);

% Wykres porównawczy
figure;
plot(y_ref_cmp, 'r', 'DisplayName','Referencja MATLAB'); hold on;
plot(data_cmp, 'b', 'DisplayName','STM32'); 
xlabel('Próbka n');
ylabel('Amplituda');
title('Porównanie wyników filtra FIR: MATLAB vs STM32');
legend;
grid on;
%% 7. Generowanie plików .h i .c dla STM32
% FIR_COEFFS
fid_h = fopen('FIR_COEFFS.h','w');
fprintf(fid_h, '#ifndef FIR_COEFFS_H_\n#define FIR_COEFFS_H_\n\n');
fprintf(fid_h, '#include "arm_math.h"\n\n');  
fprintf(fid_h, '#define NUM_TAPS %d\n', length(b));
fprintf(fid_h, 'extern float32_t FIR_COEFFS[NUM_TAPS];\n\n');
fprintf(fid_h, '#endif // FIR_COEFFS_H_\n');
fclose(fid_h);

fid_c = fopen('FIR_COEFFS.c','w');
fprintf(fid_c, '#include "FIR_COEFFS.h"\n\n');
fprintf(fid_c, 'float32_t FIR_COEFFS[NUM_TAPS] = {\n');
for i = 1:length(b)
    if i == length(b)
        fprintf(fid_c, '  %.8ff\n', b(i)); 
    else
        fprintf(fid_c, '  %.8ff,\n', b(i));
    end
end
fprintf(fid_c, '};\n');
fclose(fid_c);

% TEST_SIGNAL
fid_h = fopen('TEST_SIGNAL.h','w');
fprintf(fid_h, '#ifndef TEST_SIGNAL_H_\n#define TEST_SIGNAL_H_\n\n');
fprintf(fid_h, '#include "arm_math.h"\n\n');  
fprintf(fid_h, '#define SIGNAL_LENGTH %d\n', length(x));
fprintf(fid_h, 'extern float32_t TEST_SIGNAL[SIGNAL_LENGTH];\n\n');
fprintf(fid_h, '#endif // TEST_SIGNAL_H_\n');
fclose(fid_h);

fid_c = fopen('TEST_SIGNAL.c','w');
fprintf(fid_c, '#include "TEST_SIGNAL.h"\n\n');
fprintf(fid_c, 'float32_t TEST_SIGNAL[SIGNAL_LENGTH] = {\n');
for i = 1:length(x)
    if i == length(x)
        fprintf(fid_c, '  %.8ff\n', x(i));
    else
        fprintf(fid_c, '  %.8ff,\n', x(i));
    end
end
fprintf(fid_c, '};\n');
fclose(fid_c);

% REF_SIGNAL
fid_h = fopen('REF_SIGNAL.h','w');
fprintf(fid_h, '#ifndef REF_SIGNAL_H_\n#define REF_SIGNAL_H_\n\n');
fprintf(fid_h, '#include "arm_math.h"\n\n');  
fprintf(fid_h, '#define SIGNAL_LENGTH %d\n', length(y_ref));
fprintf(fid_h, 'extern float32_t REF_SIGNAL[SIGNAL_LENGTH];\n\n');
fprintf(fid_h, '#endif // REF_SIGNAL_H_\n');
fclose(fid_h);

fid_c = fopen('REF_SIGNAL.c','w');
fprintf(fid_c, '#include "REF_SIGNAL.h"\n\n');
fprintf(fid_c, 'float32_t REF_SIGNAL[SIGNAL_LENGTH] = {\n');
for i = 1:length(y_ref)
    if i == length(y_ref)
        fprintf(fid_c, '  %.8ff\n', y_ref(i));
    else
        fprintf(fid_c, '  %.8ff,\n', y_ref(i));
    end
end
fprintf(fid_c, '};\n');
fclose(fid_c);

%% 8. Przenoszenie plików do Components
cFiles = dir('*.c');
for i = 1:length(cFiles)
    movefile(cFiles(i).name, fullfile(srcDest, cFiles(i).name));
end

csvFiles = dir('*.csv');
for i = 1:length(csvFiles)
    movefile(csvFiles(i).name, fullfile(srcDest, csvFiles(i).name));
end

hFiles = dir('*.h');
for i = 1:length(hFiles)
    movefile(hFiles(i).name, fullfile(incDest, hFiles(i).name));
end
