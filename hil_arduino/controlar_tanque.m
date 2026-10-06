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

z = [-0.002397 -0.001];
p = [0 -10 -10];

% El período de muestreo es de 1 Hz, por lo que se traduce en 6.28 rad/seg
% más o menos, por Nyquist nuestra wc tiene que ser como mucho la mitad de
% eso, 3.14 rad/seg
kc = -5000;

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
C_tf = tf(Cz);
C_tf

[b, a] = tfdata(tf(Cz), 'v');
fprintf('u[k] = %.6f*u[k-1] + %.6f*e[k] + %.6f*e[k-1]\n', -a(2), b(1), b(2));