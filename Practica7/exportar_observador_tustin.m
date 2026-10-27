function exportar_observador_tustin
% Tustin del observador continuo en estados fisicos [theta; omega].
% G: grados/us. Arduino: rad y rad/s. Retardo externo: dos muestras.
carpeta = fileparts(mfilename('fullpath'));
% Modelo corregido del TP2 (Codigos Matlab/disenar_control.m y su informe).
% Entrada: variacion del pulso [us]. Salida: variacion del angulo [grados].
% No usar modelo_estimado.mat: corresponde a la identificacion anterior.
G = tf(36.82, [1 50.12 967.5]);
% El retardo se aplica en Arduino; no esta incluido en esta transferencia.
[num, den] = tfdata(G, 'v');
num = num/den(1); den = den/den(1);
assert(numel(den)==3 && all(num(1:end-1)==0), ...
    'Se requiere segundo orden y numerador constante.');
Ts = 0.02;
Ac = [0 1; -den(3) -den(2)];
Bc = [0; num(end)*pi/180];
C = [1 0];
% Parametros de polos continuos del exportador original (sin su override z).
omega_n_obs = 2*max(abs(real(eig(Ac))));
zeta_obs = 0.58;
p_cont = omega_n_obs*[-zeta_obs+(1.25)*1j*sqrt(1-zeta_obs^2); ...
                       -zeta_obs-(1.25)*1j*sqrt(1-zeta_obs^2)];
assert(rank(obsv(Ac,C))==2, 'Planta no observable.');
Lc = real(acker(Ac', C', p_cont)');
F = Ac-Lc*C;
h = Ts/2;
M = eye(2)-h*F;
% Estado fisico, sin la transformacion de coordenadas de c2d(ss,...,'tustin').
Ad = M\(eye(2)+h*F);
Bu = M\(h*Bc);
By = M\(h*Lc);
p_tustin = (1+h*p_cont)./(1-h*p_cont);
assert(all(abs(eig(Ad))<1), 'Observador inestable.');
assert(max(abs(poly(Ad)-poly(p_tustin)))<1e-8, 'Polos inconsistentes.');
% Filtro complementario continuo: theta_dot=gyro+(theta_acc-theta)/tau.
% tau corresponde al decaimiento alpha=0.94 a Ts=20 ms.
tau = -Ts/log(0.94);
filtroA = (2*tau-Ts)/(2*tau+Ts);
filtroAcc = Ts/(2*tau+Ts);
filtroGyro = tau*Ts/(2*tau+Ts);
valores = [Ad(1,1) Ad(1,2) Ad(2,1) Ad(2,2) Bu' By' ...
    filtroA filtroAcc filtroGyro];
assert(all(isfinite(valores)), 'Coeficientes no finitos.');
ruta = fullfile(carpeta, 'validar_observador_tustin', 'coeficientes_observador_tustin.h');
fid = fopen(ruta,'w');
assert(fid>=0, 'No se pudo escribir el encabezado.');
limpieza = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'// Generado por exportar_observador_tustin.m\n#pragma once\n');
nombres = {'OBS_A11','OBS_A12','OBS_A21','OBS_A22', ...
    'OBS_BU1','OBS_BU2','OBS_BY1','OBS_BY2', ...
    'FILTRO_A','FILTRO_ACC','FILTRO_GYRO'};
for i=1:numel(valores)
    fprintf(fid,'const float %s = %.9ef;\n', nombres{i}, valores(i));
end
disp('Ganancia continua L:'); disp(Lc);
disp('Polos Tustin del observador:'); disp(eig(Ad));
fprintf('Generado: %s\n',ruta);
end
