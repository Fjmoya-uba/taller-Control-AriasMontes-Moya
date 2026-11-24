clear; clc; close all;

%% Modelo de la planta
% Estados: x = [theta; omega; posicion; velocidad]
% Unidades: rad, rad/s, cm, cm/s
% Entrada: variacion del pulso respecto al equilibrio [us]

Ts = 0.02;          % Muestreo: 50 Hz

% Servo y barra
a0 = 967.5;
a1 = 50.12;
b0 = 36.82*pi/180;

% Carrito
a_carrito = 3.2683171;
k_carrito = 13.487667*180/pi;

A = [ 0           1       0       0;
     -a0         -a1      0       0;
      0           0       0       1;
      k_carrito   0       0      -a_carrito ];

B = [0; b0; 0; 0];

% Salida que debe seguir la referencia: posicion del carrito
C_p = [0 0 1 0];
D = 0;

%% Sistema aumentado: xi_dot = p_ref - p
% u = -K*xhat + Ki*xi. No se agrega feedforward.
Aa = [A zeros(4,1); -C_p 0];
Ba = [B; 0];
Er = [zeros(4,1); 1];
polos_controlador = [-4 -8 -20 -25 -3];
Ka = place(Aa, Ba, polos_controlador)
K = Ka(1:4);
Ki = -Ka(5);
disp('K y Ki (u = -K*xhat + Ki*xi):'); disp(K); disp(Ki);
disp('Polos aumentados:'); disp(eig(Aa-Ba*Ka));



%% Exportacion al sketch independiente
carpeta_script = fileparts(mfilename('fullpath'));
carpeta_arduino = fullfile(carpeta_script, '..', 'Arduino', 'controlador_integral');
if ~isfile(fullfile(carpeta_arduino, 'controlador_integral.ino'))
    carpeta_arduino = uigetdir(pwd, 'Seleccionar TP3/Arduino/controlador_integral');
    if isequal(carpeta_arduino, 0), return; end
end
assert(isfile(fullfile(carpeta_arduino, 'controlador_integral.ino')), 'Carpeta incorrecta.');
fid = fopen(fullfile(carpeta_arduino, 'coeficientes_controlador.h'), 'w');
assert(fid ~= -1, 'No se pudo abrir el encabezado.');
cerrar = onCleanup(@() fclose(fid));
fprintf(fid, '// Generado por controlador_integral.m. u = -K*xhat + Ki*xi.\n');
fprintf(fid, '#pragma once\n');
fprintf(fid, 'const float CONTROL_TS = %.12ef;\n', Ts);
fprintf(fid, 'const float CONTROL_K[4] = {%.12ef, %.12ef, %.12ef, %.12ef};\n', K);
fprintf(fid, 'const float CONTROL_KI = %.12ef;\n', Ki);
clear cerrar;
