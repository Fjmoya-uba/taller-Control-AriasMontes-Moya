%% Creamos el modelo de la dinámica externa
clear; clc; close all;

s = tf("s");

K_fisico =  981 * (pi/180); % ya que paso de grados a centímetros

H0 = K_fisico * 1/s^2;

bode(H0); grid on;

% Tenemos -180° siempre, lo que vamos a buscar es poner una red de adelanto
% con un margen de fase 60° aprox y tmb.

%% Armamos el controlador

% Vamos a hacer un PD + filtro.

z = [-0.7];
p = [-25];
k = db2mag(9.03);

C = zpk(z,p,k);

CP = minreal(H0*C);
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

%% Discretización del controlador
Ts = 0.02; % 20 ms: periodo de control del Arduino (sonar cada 60 ms).
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
