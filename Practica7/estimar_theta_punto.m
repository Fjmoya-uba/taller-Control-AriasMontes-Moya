%% Cargar el modelo estimado previamente
clear; clc; close all;

% Carga el archivo .mat que generó el script de estimación
load('modelo_estimado.mat'); 

% Período de muestreo
Ts = 0.02;

% Extraer numerador y denominador de G
[num, den] = tfdata(G, 'v');

% Si den = [1, a1, a0] y num = [K] (o [0, 0, K]):
a1 = den(2);
a0 = den(3);
b0 = num(end);

% Espacio de estados en variables físicas:
A_cont = [ 0,   1;
         -a0, -a1 ];

B_cont = [ 0;
          b0 ];

C_cont = [ 1, 0 ]; % Mide ángulo (theta)
D_cont = 0;

% Chequeamos la obervabilidad del sistema.
Om = obsv(A_cont, C_cont);

I = [1, 0; 0, 1];

% Discretizamos las matrices con la aproximación de Euler hacia adelante.

A = I + A_cont * Ts;
B = B_cont * Ts;
C = C_cont;
%% Vamos a plantear esto de los observadores de Luenberg

polos_planta = eig(A_cont);

% Ok, lo polos de la planta están en -45.53 +- 15.38i

factor = 2;
p_obs_cont = factor * real(polos_planta);
p_obs = exp(p_obs_cont * Ts);

%p_obs = [0.18 0.23]
% Cálculo de la matriz de ganancia del observador
L = acker(A', C', p_obs)';

% Nos fijamos que los autovalores estén en el círculo unitario.
autoval = eig(A - L*C);
%%
% Definís las ganancias directamente a mano:
L = [0.3; 1.2]; % Probá con estos valores iniciales

% Y dejás el chequeo de estabilidad para que avise si nos pasamos:
assert(all(abs(eig(A - L*C)) < 1), '¡Atento, con estos L el observador es inestable!');
