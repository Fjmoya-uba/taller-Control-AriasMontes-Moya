% 1. Definir planta y período de muestreo
s = tf('s');
P = -0.004233 / (s + 0.002397);
Ts = 1;

% 2. Opciones de diseño: fijar ancho de banda seguro para Ts = 1s
wc_deseado = 0.6; % rad/s (muy por debajo de Nyquist = 3.14 rad/s)
opt = pidtuneOptions('PhaseMargin', 60);

% 3. Sintonizar directamente en tiempo continuo o discreto
% Para un controlador tipo PI:
[C_pi, info] = pidtune(P, 'PI', wc_deseado, opt);

L = minreal(C_pi * P);
bode(L); grid on;

%% Mostramos el resultado continuo
C_pi
info

% 4. Discretizar por Tustin
Cz = c2d(C_pi, Ts, 'tustin');

% 5. Obtener los coeficientes para la ecuación en diferencias
[num, den] = tfdata(tf(Cz), 'v');
fprintf('alpha_1 = %.6f\n', -den(2));
fprintf('alpha_2 = %.6f\n', num(1));
fprintf('alpha_3 = %.6f\n', num(2));

Cz = tf(Cz);