%% Cargar el modelo estimado previamente
clear; clc; close all;

% Carga el archivo .mat que generó el script de estimación
load('modelo_estimado.mat'); 

% A partir de acá ya tenés disponibles en el Workspace:
% G, Gz, K, p1, p2, u0, y0, etc.
 
s = tf("s");

% Bueno, busco hacer un control en cascada asi que primero veamos que
% tenemos en la dinámica interna.

bode(G); grid on;

% Okay, tiene una ganancia de -28 db, lo que voy a buscar es que la barra
% pueda responder lo suficientemente rápido para controlar al carrito.
% Vamos a buscar que el servo responda en unos 0.25 segundos.

%% Bueno, nos ponemos a armar el controlador

% Vamos a plantear un PI + filtro pasabajos a partir de -25
z = [];
p = [0];
k = db2mag(50);

C = zpk(z,p,k);

CP = minreal(G*C);
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
