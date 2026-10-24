clear variables;

sin_delay = load('modelo_estimado_sin_retardo.mat');
delay_1m = load('modelo_estimado_delay_1m.mat');

G_sin_delay = sin_delay.G;
G_delay_1m = delay_1m.G;

%% carga de datos
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

u0 = mean(u(1:n_est));
y0 = mean(y(1:n_est));
du = u-u0;
dy = y-y0;

%% graficos

y_sin_delay = lsim(G_sin_delay, du, t_sim, [], 'zoh') + y0;
y_delay_1m = lsim(G_delay_1m, du, t_sim, [], 'zoh') + y0;

figure();
plot(t, (u-u0)/100);
hold on;
plot(t, y);
hold on;
plot(t, y_sin_delay, 'r--', t, y_delay_1m, 'b--');
legend('Acción de control', 'Salida real', 'Respuesta sin retardo', 'Respuesta con retardo de 1 muestra');
grid on;
xlabel('Tiempo [s]');
ylabel('Salidas');