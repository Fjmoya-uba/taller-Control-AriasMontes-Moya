%% Cargar el modelo estimado previamente
clear; clc; close all;

% Carga el archivo .mat que generó el script de estimación
load('modelo_estimado.mat'); 

% A partir de acá ya tenés disponibles en el Workspace:
% G, Gz, K, p1, p2, u0, y0, etc.

%% Modelización de la planta total

% Le agrego los dos ceros en el origen de la dinámica con el carrito
s = tf("s");

% Como yo mido la posición a través del sensor, y como al controlar
% la inclinación de la barra esta se traduce en una acelaración por la
% gravedad, el carrito impone una dinámica externa de dos polos en el
% origen con respecto a los otros polos complejos que tenía la dinámica
% interna del servo-barra.

K_fisico =  981 * (pi/180); % ya que paso de grados a centímetros

H0 = G * K_fisico * 1/s^2;

% Ahora, el fabricante impone un tiempo de muestreo de la pocisión cada 60
% ms aproximadamente, esto se aproxima por pade como un atraso de fase

%Pade = (1-s*0.06/2)/(1+s*0.06/2);
%Pade

% Probamos sin Pade de momento.

H = H0;

bode(H); grid on;


% Tiene un margen de fase de -1.75° aprox y el cruce por 0 db es en 0.8
% rad/seg aprox.

% Lo que voy a buscar es poner una red de adelanto para levantar la fase,
% quiero poner un cero lento para que la fase pueda aumentar alrededor de 1
% rad/seg
%% Vamos con el controlador

z = [-0.3 -0.5]; % el tiempo de asentamiento es wc/4 aprox.
p = [-20 0];

C = zpk(z, p, 1);

CP = minreal(H*C);
bode(CP); grid on;


%% Incluyo la ganancia que antes aplique a CP.
C_total = C * db2mag(37.5);

CP = minreal(H*C_total);
bode(CP); grid on;

% Ahora busco que wc sea 2 rad/seg, ya que tengo una respuesta de 2
% segundos para que el carrito vaya de un extremo de la barra al centro

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
C_pid = pid(C_total);
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
Tf = C_pid.Tf;

fprintf('\n--- Ganancias del controlador ---\n');
fprintf('Kp = %.6f\nKi = %.6f\nKd = %.6f\nTf = %.6f s\n', Kp, Ki, Kd, Tf);

% Estas ganancias describen el controlador continuo; ahora lo discretizo.
Ts = 0.02; % 20 ms: periodo de control del Arduino 
Cz = c2d(C_total, Ts, 'tustin');
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