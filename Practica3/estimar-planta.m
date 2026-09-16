%% =========================================================
%  IDENTIFICACION DE LA PLANTA
%
%  Modelo continuo supuesto:
%
%              p1*p2*c
%  G(s) = -------------------
%          (s+p1)(s+p2)
%
%  Aproximacion discreta utilizada:
%
%  y[k] = a1*y[k-1] + a2*y[k-2] + b0*u[k]
%
%% =========================================================

clear;
clc;
close all;


%% 1. CARGAR DATOS

load("mediciones_y_u.mat");

% Primero miramos qué cargó MATLAB
whos


%% 2. EXTRAER LAS SEÑALES
%
% Si al cargar el archivo aparece la variable "out",
% usamos:

y = double(out.d1(:));       % salida: ángulo IMU
u = double(out.d2(:));       % entrada
t = double(out.tout(:));     % tiempo


%% 3. VERIFICACIONES

fprintf("Cantidad de muestras y: %d\n", length(y));
fprintf("Cantidad de muestras u: %d\n", length(u));
fprintf("Cantidad de muestras t: %d\n", length(t));

if length(y) ~= length(u) || length(y) ~= length(t)
    error("u, y y t no tienen la misma cantidad de muestras");
end

% Período de muestreo
Ts = mean(diff(t));

fprintf("Ts = %.8f s\n", Ts);
fprintf("fs = %.2f Hz\n", 1/Ts);


%% 4. GRAFICAR LOS DATOS ORIGINALES

figure;

subplot(2,1,1);
plot(t, u);
grid on;
xlabel("Tiempo [s]");
ylabel("u[k]");
title("Entrada");

subplot(2,1,2);
plot(t, y);
grid on;
xlabel("Tiempo [s]");
ylabel("Ángulo");
title("Salida IMU");


%% =========================================================
% 5. ARMAR LA REGRESION
%
% Modelo:
%
% y[k] = a1*y[k-1] + a2*y[k-2] + b0*u[k]
%
% Lo escribimos:
%
% Y = X * theta
%
% donde:
%
% theta = [a1; a2; b0]
%
%% =========================================================

Y = y(3:end);

X = [ ...
    y(2:end-1), ...     % y[k-1]
    y(1:end-2), ...     % y[k-2]
    u(3:end)     ...    % u[k]
];


%% 6. MINIMOS CUADRADOS

% En MATLAB:
%
% theta = X \ Y
%
% resuelve el problema de mínimos cuadrados.

theta = X \ Y;

a1 = theta(1);
a2 = theta(2);
b0 = theta(3);

fprintf("\n===== MODELO DISCRETO =====\n");
fprintf("a1 = %.10f\n", a1);
fprintf("a2 = %.10f\n", a2);
fprintf("b0 = %.10f\n", b0);

fprintf("\ny[k] = %.6f y[k-1] + %.6f y[k-2] + %.6f u[k]\n", ...
    a1, a2, b0);


%% =========================================================
% 7. RECUPERAR LOS COEFICIENTES CONTINUOS
%
% Definimos:
%
% A = p1 + p2
% B = p1*p2
% C = p1*p2*c
%
% Modelo continuo:
%
%              C
% G(s) = ---------------
%          s^2 + A*s + B
%
%% =========================================================

D = -1/a2;

A = (a1*D - 2)/Ts;

B = (D - 1 - A*Ts)/(Ts^2);

C = b0*D/(Ts^2);

fprintf("\n===== COEFICIENTES CONTINUOS =====\n");
fprintf("A = p1 + p2 = %.10f\n", A);
fprintf("B = p1*p2   = %.10f\n", B);
fprintf("C = p1*p2*c = %.10f\n", C);


%% 8. OBTENER p1, p2 Y c

discriminante = A^2 - 4*B;

p1 = (A + sqrt(discriminante))/2;
p2 = (A - sqrt(discriminante))/2;

c = C/B;

fprintf("\n===== PARAMETROS DE LA PLANTA =====\n");
fprintf("p1 = %.10f\n", p1);
fprintf("p2 = %.10f\n", p2);
fprintf("c  = %.10f\n", c);


%% 9. CONSTRUIR G(s)

num = p1*p2*c;
den = [1, p1+p2, p1*p2];

G = tf(num, den);

fprintf("\n===== PLANTA CONTINUA ESTIMADA =====\n");
G


%% 10. VERIFICAR LA REGRESION

% Predicción de un paso usando datos medidos anteriores
y_pred = X*theta;

figure;
plot(t(3:end), Y, "DisplayName", "Medición");
hold on;
plot(t(3:end), y_pred, "--", "DisplayName", "Predicción");
grid on;
xlabel("Tiempo [s]");
ylabel("Ángulo");
title("Medición vs predicción de la regresión");
legend;