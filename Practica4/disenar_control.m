%% Cargar el modelo estimado previamente
clear; clc; close all;

% Carga el archivo .mat que generó el script de estimación
load('modelo_estimado.mat'); 

% A partir de acá ya tenés disponibles en el Workspace:
% G, Gz, K, p1, p2, u0, y0, etc.

%% Analisis de la estabilidad

figure;
margin(G);
grid on;

%% 1. Vamos con el controlador
Ts_control = 0.02; 

% Cceros (vector z_c), polos (vector p_c) y la ganancia (K_c).

z_c = [];   % Cero(s)
p_c = [];     % Polo(s) (ej: integrador en z=1)
K_c = 1;       % Ganancia

Cz = zpk(z_c, p_c, K_c, Ts_control);
Cz

Ts_real = get(Gz, 'Ts');
Cz.Ts = Ts_real;

% Lazo abierto equivalente (controlador * planta)
L = minreal(Cz * Gz);
L

%% 2. Cierre de Lazo en Discreto y Conversión a Continuo


% Lazo cerrado en el dominio Z
Tz = feedback(L, 1);

% Convertir la función de transferencia cerrada de discreta a continua
% Usamos el método 'zoh' o 'tustin' para pasar Tz a un equivalente continuo Tc(s)
Tc = d2c(Tz, 'tustin'); 
Tc
% (Nota: si querés mapear solo los polos exactos como al identificar, 
% podés extraer los polos con pole(Tz), aplicar p = -log(pole(Tz))/Ts y armar el sistema, 
% pero d2c con 'tustin' o 'zoh' te da la función de transferencia continua completa).

%% 3. Ver la respuesta al escalón en continuo
figure('Name', 'Respuesta al Escalon Continuo equivalente');
step(Tc);
grid on;
title('Respuesta al Escalon en Lazo Cerrado (Dominio Continuo)');