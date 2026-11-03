clear; clc; close all;

%% Planta continua: estados [theta (rad); omega (rad/s)], entrada relativa [us]
A = [0 1; -967.5 -50.12];
B = [0; 36.82*pi/180];
C = [1 0];
D = 0;
Ts = 0.02;
polos_planta = eig(A);

%% Control continuo con integrador: q_dot = r-y, u = -K*xhat + H*q
A_amp = [A zeros(2,1); -C 0];
B_amp = [B; 0];
polos_deseados = [-20 -30 -4];
K_amp = place(A_amp, B_amp, polos_deseados);
K = K_amp(1:2)
H = -K_amp(3)

%% Planta discreta con retencion de orden cero
sys_d = c2d(ss(A,B,C,D), Ts, 'zoh');
A_d = sys_d.A
B_d = sys_d.B

