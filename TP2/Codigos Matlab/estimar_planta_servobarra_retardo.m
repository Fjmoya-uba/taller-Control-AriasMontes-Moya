%% Regresion G(s) = K*p1*p2/((s+p1)*(s+p2))
% Requiere Control System Toolbox.
clear; clc; close all;

%% 1. Datos y division temporal
S = load('mediciones_servo_planta.mat');
y = double(S.out.d1(:));       % Salida medida
u = double(S.out.d2(:));       % Entrada al servo
 t = double(S.out.tout(:));
t_inicio = 5;                 % Descartar arranque [s]
fraccion_estimacion = 0.75;    % Puede cambiarse a 0.70 o 0.80
idx = t >= t_inicio;
u = u(idx); y = y(idx); t = t(idx);
N = numel(t);
n_est = floor(fraccion_estimacion*N);
Ts = mean(diff(t));
t_sim = (0:N-1)'*Ts;

% Variaciones respecto de medias calculadas SOLO con entrenamiento.
u0 = mean(u(1:n_est));
y0 = mean(y(1:n_est));
du = u-u0;
dy = y-y0;
fprintf('Ts = %.6g s | Entrenamiento: %d | Validacion: %d muestras\n', Ts, n_est, N-n_est);

%% 2. Regresion lineal:
% retardo de 1 muestra
% dy[k] = a1*dy[k-1] + a2*dy[k-2] + b*du[k-1]
Y = dy(3:n_est);
Phi = [dy(2:n_est-1), dy(1:n_est-2), du(2:n_est-1)];
theta = Phi\Y;
a1 = theta(1);
a2 = theta(2);
b = theta(3);

%% 3. Modelo discreto: polos y ganancia estatica
% Gz(z) = b/(z^2-a1*z-a2), por lo tanto K = Gz(1).
% (u'*y)/(u'*u) NO es la ganancia DC en general.
z_polos = roots([1, -a1, -a2]);

K = b/(1-a1-a2);
Gz = tf(b, [1, -a1, -a2], Ts);
Gz


%% 4. Planta continua con la forma pedida
% z = exp(-p*Ts) => p = -log(z)/Ts. Los polos continuos son -p1 y -p2.
% Conservamos polos y ganancia DC. 
p = -log(z_polos)/Ts;
den = [1, sum(p), prod(p)];

p1 = p(1);
p2 = p(2);
G = tf(real(K*p1*p2), real(den));
fprintf('\nG(s) = K*p1*p2 / ((s+p1)*(s+p2))\n');
fprintf('K = %.8g\np1 = %.8g %+.8gi\np2 = %.8g %+.8gi\n', ...
    K, real(p1), imag(p1), real(p2), imag(p2));

disp('Planta continua (variaciones respecto de u0 e y0):');
G

G_ret = G;
G_ret.InputDelay = 0.02;

y_cont = lsim(G, du, t_sim, [], 'zoh') + y0;
y_ret  = lsim(G_ret, du, t_sim, [], 'zoh') + y0;
%% 5. Simulacion libre y validacion en el tramo reservado
% Estado inicial cero en variables centradas para ambos modelos.
% Simulamos desde el inicio para llegar al corte con historia de entrada.
y_dis  = lsim(Gz, du, t_sim) + y0;
y_cont = lsim(G, du, t_sim, [], 'zoh') + y0;
y_ret  = lsim(G_ret, du, t_sim, [], 'zoh') + y0; % Simulación con retardo

k_val = (n_est+1:N)';

% Agrupamos los errores de los tres modelos: [Gz, G(s) sin retardo, G(s) con retardo]
errores = [y(k_val) - y_dis(k_val), ...
           y(k_val) - y_cont(k_val), ...
           y(k_val) - y_ret(k_val)];

rmse = sqrt(mean(errores.^2, 1));
den_fit = norm(y(k_val) - mean(y(k_val)));
fit = [NaN, NaN, NaN];
if den_fit > eps
    fit = 100 * (1 - sqrt(sum(errores.^2, 1)) / den_fit);
end

fprintf('\nValidacion: ultimo %.0f %% (sin usarlo en la regresion)\n', 100*(1-fraccion_estimacion));
fprintf('G(z):     RMSE = %.6g | FIT = %.2f %%\n', rmse(1), fit(1));
fprintf('G(s):     RMSE = %.6g | FIT = %.2f %%\n', rmse(2), fit(2));
fprintf('G(s) + ret: RMSE = %.6g | FIT = %.2f %%\n', rmse(3), fit(3));
%% 7. Guardo el modelo
save('modelo_estimado_delay_1m.mat', 'G');
%%
s = tf('s');
step(100*G*exp(-0.04*s));

%% Visualización de los datos
figure('Name', 'Datos de Identificacion y Validacion', 'Color', 'white');

% Gráfico de la Entrada (u)
subplot(2,1,1);
plot(t, u, 'LineWidth', 1.2);
hold on;
% Línea vertical que marca el fin de la estimación y el inicio de la validación
xline(t(n_est), '--r', 'LineWidth', 1.5, 'DisplayName', 'Límite Estimación / Validación');
grid on;
xlabel('Tiempo [s]');
ylabel('Entrada u(t)');
title('Señal de Entrada al Servo');
legend('Location', 'best');

% Gráfico de la Salida (y)
subplot(2,1,2);
plot(t, y, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.2);
hold on;
xline(t(n_est), '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid on;
xlabel('Tiempo [s]');
ylabel('Salida y(t)');
title('Salida Medida del Sistema');



%% 6. Gráficos comparativos unificados
figure('Name', 'Comparación del Modelo vs Medición Real', 'Color', 'white');

% Gráfico superior: Las tres señales juntas en el tramo completo (o en validación)
subplot(2,1,1);
plot(t, y, 'k', 'LineWidth', 1.2); hold on;               % Salida real / filtro
plot(t, y_cont, 'b--', 'LineWidth', 1.2);                 % Modelo por regresión
xline(t(n_est+1), ':r', 'Inicio Validación', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid on;
ylabel('Salida y [grados]'); 
legend('Medida Real / Filtro', 'Modelo Regresión G(s)', 'Modelo tfest G(s)', 'Location', 'best');
title('Comparación de Modelos Identificados vs Salida Real');

% Gráfico inferior: Errores de validación de ambos modelos
subplot(2,1,2);
plot(t(k_val), y(k_val) - y_cont(k_val), 'b', 'LineWidth', 1.2); hold on;
grid on;
xlabel('Tiempo [s]'); 
ylabel('Error de Predicción');
legend('Error Modelo Regresión', 'Error Modelo tfest', 'Location', 'best');
title('Evolución del Error en el Tramo de Validación');