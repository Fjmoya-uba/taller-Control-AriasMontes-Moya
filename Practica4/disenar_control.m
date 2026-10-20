%% Cargar el modelo estimado previamente
clear; clc; close all;

% Carga el archivo .mat que generó el script de estimación
%load('modelo_estimado.mat'); 
%load('dinamica_carrito_estimada.mat');

p1 = [-45.5373-15.3815i -45.5373+15.3815i];
z1 = [];
k1 = 87.92;

G_barra = zpk(z1,p1,k1)

p2 = [0 -8.4596013];
z2 = [];
k2 = 50.389071;

G_carrito = zpk(z2,p2,k2)

%% Modelización de la planta total

s = tf("s");

Pade = (1-s*0.02/2)/(1+s*0.02/2);


% Probamos sin Pade de momento.

H = minreal(G_barra*G_carrito);
H

%bode(H); grid on;


%% Vamos con el controlador

z = [-1.2 -10]; % el tiempo de asentamiento es 4/wc aprox.
p = [0 -20];
k = db2mag(26);
C = zpk(z, p, k);

CP = minreal(H*C);
bode(CP); grid on;


%% Lazo cerrado de posición
T = feedback(CP, 1);

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
xlabel('Tiempo [s]');

%% Ganancias y discretizacion del controlador

% La red de adelanto equivale a un PD con derivada filtrada:
% C_total(s) = Kp + Ki/s + Kd*s/(Tf*s + 1). En este caso Ki = 0.
C_pid = pid(C);
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
Tf = C_pid.Tf;

fprintf('\n--- Ganancias del controlador ---\n');
fprintf('Kp = %.6f\nKi = %.6f\nKd = %.6f\nTf = %.6f s\n', Kp, Ki, Kd, Tf);

% Estas ganancias describen el controlador continuo; ahora lo discretizo.
Ts = 0.02; % 20 ms: periodo de control del Arduino 
Cz = c2d(C, Ts, 'tustin');
Cz

[b, a] = tfdata(tf(Cz), 'v');

if length(a) == 2
    % Primer orden (PD o red de adelanto simple)
    fprintf('u[k] = %.6f*u[k-1] + %.6f*e[k] + %.6f*e[k-1]\n', ...
            -a(2), b(1), b(2));
elseif length(a) == 3
    % Segundo orden (PID o adelanto + integrador)
    fprintf('u[k] = %.6f*u[k-1] + %.6f*u[k-2] + %.6f*e[k] + %.6f*e[k-1] + %.6f*e[k-2]\n', ...
            -a(2), -a(3), b(1), b(2), b(3));
end