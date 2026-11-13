clear; clc; close all;

%% Planta continua: estados [theta (rad); omega (rad/s)], entrada relativa [us]
A = [0 1; -967.5 -50.12];
B = [0; 36.82*pi/180];
C = [1 0];
D = 0;
Ts = 0.02;
polos_planta = eig(A);

%% Realimentacion de estados con seguimiento de referencia
% u = -K*xhat + Fr*r
polos_referencia = [-20 -30];
K_ref = place(A, B, polos_referencia);

% Ganancia para que y = r en regimen permanente (D = 0)
Fr = -1 / (C * ((A - B*K_ref) \ B))



%% Control continuo con integrador: q_dot = r-y, u = -K*xhat + H*q
A_amp = [A zeros(2,1); -C 0];
B_amp = [B; 0];
polos_integral = [-20 -30 -4];
K_amp = place(A_amp, B_amp, polos_integral);
K_int = K_amp(1:2)
H = -K_amp(3)

%% Planta discreta con retencion de orden cero
sys_d = c2d(ss(A,B,C,D), Ts, 'zoh');
A_d = sys_d.A
B_d = sys_d.B


%% Copiar estas constantes a Arduino (diseno continuo)
fprintf('K_REF_THETA = %.8ff;\n', K_ref(1));
fprintf('K_REF_OMEGA = %.8ff;\n', K_ref(2));
fprintf('FR_REFERENCIA = %.8ff;\n', Fr);
fprintf('K_INT_THETA = %.8ff;\n', K_int(1));
fprintf('K_INT_OMEGA = %.8ff;\n', K_int(2));
fprintf('H_INTEGRAL = %.8ff;\n', H);
