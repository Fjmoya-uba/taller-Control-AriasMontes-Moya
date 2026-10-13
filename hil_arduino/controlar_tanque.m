% Comenzamos
clear; clc; close all;

s = tf("s");

z = [];
p = [-0.002397];
kp = -0.004233;

P = zpk(z,p,kp);
P

bode(P); grid on;
% bueno lo que tenemos es una planta que si la realimento es inestable con
% un marge de fase de -55 grados y wc = 0.00349. Es una planta muy lenta,
% vamos a ver que es lo que podemos hacer...
%% Vamos por un controlador

z = [-0.02];
p = [0 ];

% El período de muestreo es de 1 Hz, por lo que se traduce en 6.28 rad/seg
% más o menos, por Nyquist nuestra wc tiene que ser como mucho la mitad de
% eso, 3.14 rad/seg

% PROBLEMA DE wc: Al elegir wc = 2.11 rad/s (o 4.23 rad/s) con un muestreo Ts = 1 s,
% estabas violando la regla de control discreto (ws >= 10*wc a 20*wc) y operando 
% pegado al límite de Nyquist (wN = pi/Ts = 3.14 rad/s). El retardo intrínseco de 
% muestreo (Td ~ Ts/2 = 0.5 s) introdujo un desfase destructivo (-wc*Td ~ -60° a -120°),
% comiéndose todo el margen de fase y volviendo inestable al lazo con Gm < 0 dB. 
% Para Ts = 1 s, wc no debe superar los 0.2 - 0.3 rad/s.

kc = -6.5;%db2mag(34.5);

% Propusimos un PI para empezar.
C = zpk(z,p,kc);
C

L = minreal(C*P);
L

bode(L); grid on;

%% Step en lazo cerrado
T = feedback(L, 1);


% 1. Calcular toda la información del escalón
info = stepinfo(T);

% 2. Imprimir los datos por consola de forma ordenada
fprintf('--- Caracteristicas de la Respuesta al Escalon ---\n');
fprintf('Tiempo de establecimiento (SettlingTime): %.4f s\n', info.SettlingTime);
fprintf('Sobrepaso máximo (Overshoot): %.2f %%\n', info.Overshoot);
fprintf('Valor pico (Peak): %.4f\n', info.Peak);
fprintf('Tiempo al pico (PeakTime): %.4f s\n', info.PeakTime);
fprintf('Tiempo de subida (RiseTime): %.4f s\n', info.RiseTime);

% 3. Graficar la respuesta y opcionalmente mostrar las características visualmente
figure;
step(T); grid on;
title('Respuesta al Escalón en Lazo Cerrado');
ylim([0 1.1]);
xlim([0 5]);
xlabel('Tiempo [s]');

%% Ahora nos toca discretizar por metodo bilineal y hacer la ecuación en diferencias.

% Periodo de muestreo
Ts = 1;
Cz = c2d(C, Ts, 'tustin');
Cz

% Convertir el modelo zpk a tf
Cz = tf(Cz);
Cz

[b, a] = tfdata(tf(Cz), 'v');
fprintf('u[k] = %.6f*u[k-1] + %.6f*e[k] + %.6f*e[k-1]\n', -a(2), b(1), b(2));

%% Discretizamos planta y controlador con Ts = 1
Pd = c2d(P, 1, 'zoh');
Cd = c2d(C, 1, 'tustin');
Ld = Cd * Pd;

% Mirá el margen de fase real en discreto:
margin(Ld); grid on;