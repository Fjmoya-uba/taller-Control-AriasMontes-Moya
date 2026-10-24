%% Comparar la medicion real con dos transferencias distintas
% Ambas transferencias se simulan con el MISMO tren de impulsos medido.
% No se ajustan parametros: solamente se simulan los modelos dados.
% Requiere Control System Toolbox.
clear; clc; close all;

%% 1. Opciones y modelos a comparar (EDITAR)
archivo = 'mediciones_dinamica_carrito_2.mat';
t_inicio = 5;                 % Descartar arranque [s]
fraccion_estimacion = 0.75;   % Mismo corte que en la identificacion

% --- Transferencia 1: G(s) = K*p1*p2/((s+p1)*(s+p2)), sin retardo ---
K1 = 0.0379; p1_a = 18.7714 - 13.9467i; p1_b = 18.7714 + 13.9467i;                 % <-- poner los valores estimados
G1 = tf(K1*p1_a*p1_b, [1, p1_a+p1_b, p1_a*p1_b]);
nombre1 = 'G_1(s) sin retardo';

% --- Transferencia 2: otra G(s) (otros parametros, otro orden, con retardo...) ---
K2 = 0.0381; p2_a = 25.0609 - 18.4242i; p2_b = 25.0609 + 18.4242i;                 % <-- poner los valores estimados
G2 = tf(K2*p2_a*p2_b, [1, p2_a+p2_b, p2_a*p2_b]);
%G2.InputDelay = 0.02;                         % [s]; borrar si no hay retardo
nombre2 = 'G_2(s) con retardo 0.02 s';

%% 2. Datos y division temporal
S = load(archivo);
%angulo = double(S.out.d1(:));
y = double(S.out.d2(:));   % posición del carrito [cm]
u = double(S.out.d3(:)); % Variacion del pulso [us]
t = double(S.out.tout(:));
assert(numel(t)==numel(y) && numel(t)==numel(u), ...
    'Los vectores tienen longitudes distintas.');

idx = t >= t_inicio;
u = u(idx); y = y(idx); t = t(idx);
N = numel(t);
n_est = floor(fraccion_estimacion*N);
Ts = mean(diff(t));
t_sim = (0:N-1)'*Ts;
k_val = (n_est+1:N)';

% Variaciones respecto de medias calculadas SOLO con el tramo de estimacion
u0 = mean(u(1:n_est));
y0 = mean(y(1:n_est));
du = u - u0;

fprintf('Ts = %.6g s | %d muestras | validacion: %d muestras\n', ...
    Ts, N, numel(k_val));

%% 3. Simular ambos modelos con la misma entrada
modelos = {G1, G2};
nombres = {nombre1, nombre2};
y_sim = zeros(N, 2);
rmse  = zeros(2, 1);
fit   = NaN(2, 1);

den_fit = norm(y(k_val) - mean(y(k_val)));
for i = 1:2
    y_sim(:,i) = lsim(modelos{i}, du, t_sim, [], 'zoh');% + y0;
    e = y(k_val) - y_sim(k_val,i);
    rmse(i) = sqrt(mean(e.^2));
    if den_fit > eps
        fit(i) = 100*(1 - norm(e)/den_fit);
    end
    fprintf('%s: RMSE = %.5g | FIT = %.2f %%\n', nombres{i}, rmse(i), fit(i));
end
resultados = table(nombres(:), rmse, fit, ...
    'VariableNames', {'Modelo', 'RMSE', 'FIT_porcentaje'});
[~, mejor] = min(rmse);
fprintf('Menor RMSE en validacion: %s\n', nombres{mejor});

%% 4. Graficos
figure('Name', 'Medicion real vs dos transferencias', 'Color', 'white');

subplot(2,1,1);
plot(t, du/100, 'LineWidth', 1.1); hold on;
plot(t, y_sim(:,1), 'b--', 'LineWidth', 1.2); hold on;
plot(t, y_sim(:,2), 'r--', 'LineWidth', 1.2);
grid on;
xline(t(n_est+1), ':k', 'Validacion', 'HandleVisibility', 'off');
ylabel('Entrada y salidas normalizadas'); title('Tren de impulsos medido y respuestas');
legend('Acción de control', 'Respuesta sin retardo', 'Respuesta con retardo');

subplot(2,1,2);
plot(t, y-y0, 'k', 'LineWidth', 1.2); hold on;
plot(t, y_sim(:,1), 'b--', 'LineWidth', 1.2); hold on;
plot(t, y_sim(:,2), 'r--', 'LineWidth', 1.2);
xline(t(n_est+1), ':k', 'Validacion', 'HandleVisibility', 'off');
grid on; xlabel('Tiempo [s]'); ylabel('Salidas normalizadas');
legend([{'Medida'}, nombres], 'Location', 'best');
title('Respuesta medida vs simulada');
