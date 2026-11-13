clear; clc; close all;

%% Planta continua
% Estados: x = [theta; velocidad angular; posicion; velocidad]
% Unidades: rad, rad/s, cm, cm/s
% Entrada: u = variacion del pulso respecto al equilibrio [us]
% Salidas: y = [theta; posicion]

Ts = 0.02;  % Muestreo a 50 Hz

% Servo-barra: modelo del TP2
a0 = 967.5;
a1 = 50.12;
b0 = 36.82*pi/180;  % Convertimos grados a radianes

% Carrito: modelo corregido elegido para el TP3
a_carrito = 3.2683171;
k_carrito = 13.487667*180/pi;  % Entrada angular en radianes

A = [ 0       1          0       0;
     -a0     -a1         0       0;
      0       0          0       1;
      k_carrito  0       0      -a_carrito ];

B = [0; b0; 0; 0];

C = [1 0 0 0;
     0 0 1 0];

D = zeros(2,1);

polos_planta = eig(A)

%% Observabilidad
% Cuatro estados: necesitamos rango 4

rango_ambas = rank(obsv(A, C))

% Solo posicion
rango_posicion = rank(obsv(A, C(2,:)))

% Solo angulo
rango_angulo = rank(obsv(A, C(1,:)))

%% Observador continuo: parte servo-barra
A_servo = A(1:2, 1:2);
C_servo = [1 0];

% Mismo criterio que en exportar_observador_tustin.m
omega_n_obs = 2*max(abs(real(eig(A_servo))));
zeta_obs = 0.58;

polos_servo = omega_n_obs * ...
    [-zeta_obs + 1.25*1i*sqrt(1-zeta_obs^2);
     -zeta_obs - 1.25*1i*sqrt(1-zeta_obs^2)];

L_servo = place(A_servo', C_servo', polos_servo)';

%% Observador continuo: parte carrito
A_carrito = A(3:4, 3:4);
C_carrito = [1 0];

omega_n_carrito = 2*max(abs(real(eig(A_carrito))));
zeta_carrito = 0.58;

polos_carrito = omega_n_carrito * ...
    [-zeta_carrito + 1.25*1i*sqrt(1-zeta_carrito^2);
     -zeta_carrito - 1.25*1i*sqrt(1-zeta_carrito^2)];

L_carrito = place(A_carrito', C_carrito', polos_carrito)';


%% Ganancia del observador completo
% Columnas: error de angulo y error de posicion
L = [L_servo       zeros(2,1);
     zeros(2,1)   L_carrito]

% Comprobamos los cuatro polos del error
polos_error = eig(A - L*C)

%% Observador discreto por Tustin, conservando los estados fisicos
% Se discretiza xhat_dot = (A-L*C)*xhat + B*u_ret + L*y.
% No usar directamente el estado de c2d(...,'tustin'): puede estar
% transformado y no representar [theta; omega; posicion; velocidad].
F = A - L*C;
h = Ts/2;
M = eye(4) - h*F;
OBS_AD = M\(eye(4) + h*F);
OBS_BU = M\(h*B);
OBS_BY = M\(h*L);

polos_discretos = eig(OBS_AD)
polos_tustin = (1 + h*polos_error)./(1 - h*polos_error);


%% Exportar matrices listas para incluir desde el sketch
% Buscar el proyecto desde el archivo y desde la carpeta actual. MATLAB
% puede ejecutar una copia temporal del Editor: no exportar junto a ella.
origenes = {fileparts(mfilename('fullpath')), pwd};
carpeta_arduino = '';
for origen = 1:numel(origenes)
    carpeta_busqueda = origenes{origen};
    while ~isempty(carpeta_busqueda)
        candidata = fullfile(carpeta_busqueda, 'TP3', 'Arduino', 'observador_completo');
        if isfolder(candidata)
            carpeta_arduino = candidata;
            break;
        end
        carpeta_padre = fileparts(carpeta_busqueda);
        if strcmp(carpeta_padre, carpeta_busqueda)
            break;
        end
        carpeta_busqueda = carpeta_padre;
    end
    if ~isempty(carpeta_arduino)
        break;
    end
end
if isempty(carpeta_arduino)
    raiz_proyecto = uigetdir(pwd, 'Seleccionar la raiz del proyecto que contiene TP3');
    assert(~isequal(raiz_proyecto, 0), 'Exportacion cancelada.');
    carpeta_arduino = fullfile(raiz_proyecto, 'TP3', 'Arduino', 'observador_completo');
end
ruta_header = fullfile(carpeta_arduino, 'coeficientes_observador_completo.h');
fid = fopen(ruta_header, 'w');
try
    fprintf(fid, '// Generado por observador_completo.m. No editar a mano.\n');
    fprintf(fid, '// Estados: theta [rad], omega [rad/s], p [cm], v [cm/s].\n');
    fprintf(fid, '// Salidas: theta [rad], p [cm]. Entrada relativa [us].\n');
    fprintf(fid, '// x[k]=AD*x[k-1]+BU*(u[k-d]+u[k-d-1])+BY*(y[k]+y[k-1]).\n');
    fprintf(fid, '#pragma once\n');
    fprintf(fid, 'const float OBS_TS = %.12ef;\n', Ts);
    fprintf(fid, 'const float OBS_AD[4][4] = {\n');
    for fila = 1:4
        fprintf(fid, '  {%.12ef, %.12ef, %.12ef, %.12ef},\n', OBS_AD(fila,:));
    end
    fprintf(fid, '};\n');
    fprintf(fid, 'const float OBS_BU[4] = {%.12ef, %.12ef, %.12ef, %.12ef};\n', OBS_BU);
    fprintf(fid, 'const float OBS_BY[4][2] = {\n');
    for fila = 1:4
        fprintf(fid, '  {%.12ef, %.12ef},\n', OBS_BY(fila,:));
    end
    fprintf(fid, '};\n');
catch error_exportacion
    fclose(fid);
    rethrow(error_exportacion);
end
fclose(fid);
% El regulador usa exactamente el mismo observador, sin copiar a mano.
carpeta_control = fullfile(carpeta_arduino, '..', 'control_sin_referencia');
if isfolder(carpeta_control)
    copyfile(ruta_header, fullfile(carpeta_control, 'coeficientes_observador_completo.h'));
end
% El controlador con referencia usa el mismo observador.
carpeta_referencia = fullfile(carpeta_arduino, '..', 'controlador_con_referencia');
if isfolder(carpeta_referencia)
    copyfile(ruta_header, fullfile(carpeta_referencia, 'coeficientes_observador_completo.h'));
end
fprintf('Coeficientes exportados: %s\n', ruta_header);
fprintf('Muestreo: %.0f Hz\n', 1/Ts);

%% Retardo externo de la entrada y ecuacion para Arduino
% Valor heredado de Practica7: 40 ms = 2 muestras. Confirmar contra TP2
% antes del ensayo. No esta incluido en A, B ni en la discretizacion.
retardo_muestras = 2;
retardo_s = retardo_muestras*Ts;
% Con d = retardo_muestras:
% xhat[k] = OBS_AD*xhat[k-1]
%         + OBS_BU*(u[k-d] + u[k-d-1])
%         + OBS_BY*(y[k] + y[k-1]).
% u: pulso REAL aplicado menos pulso de equilibrio [us].
% y: [angulo relativo en rad; posicion relativa en cm].
% Los dos extremos de u_ret son la regla trapezoidal, no otro retardo.
% Calcular los cuatro estados nuevos con una copia del vector anterior;
% reemplazar xhat solamente al terminar todas las filas.
% El historial debe respetar los instantes reales de medicion y comando.
% En equilibrio: xhat=0, historial u=0 e historial y=0.
% Si se inicia fuera del equilibrio, usar las mediciones iniciales para
% theta y posicion, velocidades inicialmente cero y y_anterior=y_inicial.

