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

%% Realimentacion de estados
% Mismos polos que en el regulador sin referencia
polos_controlador = [-4 -8 -20 -25];

K = place(A, B, polos_controlador);

%% Ganancia de seguimiento de referencia
% Ley de control:
% u = -K*xhat + Fr*p_ref
%
% p_ref se expresa en cm respecto al cero de posicion.
% Fr da ganancia estatica unitaria en el modelo nominal.

Acl = A - B*K;
Fr = -1 / (C_p * (Acl \ B));

%% Comprobar resultados
disp('Ganancias K:');
disp(K);

disp('Ganancia de referencia Fr [us/cm]:');
disp(Fr);

disp('Polos del lazo cerrado:');
disp(eig(Acl));

% Para este modelo, Fr debe coincidir con K(3)
fprintf('Fr - K_posicion = %.3e\n', Fr - K(3));

%% Exportar ganancias para Arduino
% Buscar desde el script, la carpeta actual y el archivo del Editor.
% No depender del nombre del .m ni exportar junto a una copia temporal.
origenes = {fileparts(mfilename('fullpath')), pwd};
try
    documento = matlab.desktop.editor.getActive;
    if ~isempty(documento)
        origenes{end+1} = fileparts(documento.Filename);
    end
catch
    % MATLAB sin Editor: quedan disponibles las otras rutas.
end

carpeta_arduino = '';
for i = 1:numel(origenes)
    carpeta = origenes{i};
    while ~isempty(carpeta)
        candidata = fullfile(carpeta, 'TP3', 'Arduino', 'controlador_con_referencia');
        if isfile(fullfile(candidata, 'controlador_con_referencia.ino'))
            carpeta_arduino = candidata;
            break;
        end
        % Tambien admitir que MATLAB este situado en la carpeta del sketch.
        if isfile(fullfile(carpeta, 'controlador_con_referencia.ino'))
            carpeta_arduino = carpeta;
            break;
        end
        padre = fileparts(carpeta);
        if strcmp(padre, carpeta)
            break;
        end
        carpeta = padre;
    end
    if ~isempty(carpeta_arduino)
        break;
    end
end

% Si no se encuentra el proyecto, elegir directamente la carpeta del sketch.
if isempty(carpeta_arduino)
    carpeta_arduino = uigetdir(pwd, ...
        'Seleccionar TP3/Arduino/controlador_con_referencia');
    if isequal(carpeta_arduino, 0)
        disp('Exportacion cancelada.');
        return;
    end
end
assert(isfile(fullfile(carpeta_arduino, 'controlador_con_referencia.ino')), ...
    'Elegir la carpeta que contiene controlador_con_referencia.ino.');
ruta_header = fullfile(carpeta_arduino, 'coeficientes_controlador.h');
fid = fopen(ruta_header, 'w');
assert(fid ~= -1, 'No se pudo abrir el encabezado del controlador.');
cerrar_archivo = onCleanup(@() fclose(fid));
fprintf(fid, '// Generado por controlador_con_referencia.m. No editar a mano.\n');
fprintf(fid, '// u = -K*xhat + Fr*p_ref. Estados: theta, omega, p, v. Referencia en cm.\n');
fprintf(fid, '#pragma once\n');
fprintf(fid, 'const float CONTROL_TS = %.12ef;\n', Ts);
fprintf(fid, 'const float CONTROL_K[4] = {%.12ef, %.12ef, %.12ef, %.12ef};\n', K);
fprintf(fid, 'const float CONTROL_FR = %.12ef;\n', Fr);
clear cerrar_archivo;
fprintf('Ganancias exportadas: %s\n', ruta_header);
