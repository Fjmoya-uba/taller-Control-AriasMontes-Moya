%% Cargar el modelo estimado previamente
clear; clc; close all;


%% Modelización de la planta total

p = [ 0 -6.815007];
z = [];
k = 24.428153;

G_carrito = zpk(z,p,k)

s = tf("s");
G_servo_barra = 36.82/(s^2 + 50.12*s + 967.5)

% Conecto en cascada la dinamica del servo y la del carrito.

H = minreal(G_servo_barra * G_carrito)

%%
figure();
bode(H); grid on;
title('Bode de la Planta Total');


%% Vamos con el controlador

nombre_controlador = 'PI'; % Cambiar tambien al ensayar P o PD (nombres de salida).
z = [-3]; 
p = [-6];
k = db2mag(24);
C = zpk(z, p, k);

CP = minreal(H*C);
figure(); bode(CP); grid on;
title('Bode de la Planta con un Controlador tipo PD');

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
step(10*T); grid on;
title('Respuesta continua a un escalon de 10 cm');
ylabel('Posicion [cm]');
xlabel('Tiempo [s]');

%% Ganancias y discretizacion del controlador

% Extraer ganancias del controlador continuo seleccionado.
C_pid = pid(C);
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
Tf = C_pid.Tf;

fprintf('\n--- Ganancias del controlador ---\n');
fprintf('Kp = %.6f\nKi = %.6f\nKd = %.6f\nTf = %.6f s\n', Kp, Ki, Kd, Tf);

% Estas ganancias describen el controlador continuo; ahora lo discretizo.
Ts = 0.02; % 20 ms: periodo de control del Arduino 
Cz = c2d(C, Ts, 'tustin');
Cz

[b, a] = tfdata(tf(Cz), 'v');
b = b/a(1);
a = a/a(1);
assert(numel(a)<=3 && numel(b)<=3, 'Arduino admite orden hasta dos.');

if length(a) == 2
    % Primer orden (PD o red de adelanto simple)
    fprintf('u[k] = %.6f*u[k-1] + %.6f*e[k] + %.6f*e[k-1]\n', ...
            -a(2), b(1), b(2));
elseif length(a) == 3
    % Segundo orden (PID o adelanto + integrador)
    fprintf('u[k] = %.6f*u[k-1] + %.6f*u[k-2] + %.6f*e[k] + %.6f*e[k-1] + %.6f*e[k-2]\n', ...
            -a(2), -a(3), b(1), b(2), b(3));
end

% Completar coeficientes ausentes: misma recurrencia para P, PI y PD filtrado.
b(end+1:3) = 0;
a(end+1:3) = 0;

%% Simulaciones del controlador digital: escalon y golpe
% La planta se discretiza con ZOH: Arduino mantiene el comando entre muestras.
Hz = c2d(H, Ts, 'zoh');
Tz = feedback(Cz*Hz, 1);
fprintf('\nLazo continuo estable: %d | Lazo digital estable: %d\n', ...
    isstable(T), isstable(Tz));
assert(isstable(T) && isstable(Tz), 'Revisar controlador: el lazo no es estable.');
fprintf('Arduino: A1=%.9g; A2=%.9g; B0=%.9g; B1=%.9g; B2=%.9g;\n', ...
    -a(2), -a(3), b(1), b(2), b(3));
figure('Name','Margenes digitales','Color','white');
margin(Cz*Hz); grid on;

%% Opciones de los ensayos
tiempo = (0:Ts:30)';
instante_escalon = 2;       % Referencia cero hasta t=2 s, luego 10 cm
instante_golpe = 5;
salto_velocidad = 5;        % cm/s: efecto idealizado de un golpe, configurable
pulso_centro = 1544;
pulso_min = 1100;
pulso_max = 2000;
% 0: modelo nominal. Cambiar a -3.2630965 para incluir el sesgo identificado.
c_simulacion = 0;           % cm/s^2

% Modelo de estados: posicion, velocidad, angulo, velocidad angular.
% Dos entradas: variacion de pulso y aceleracion externa constante.
planta = ss([0 1 0 0; 0 -6.815007 24.428153 0; ...
    0 0 0 1; 0 0 -967.5 -50.12], ...
    [0 0; 0 1; 0 0; 36.82 0], [1 0 0 0], [0 0]);
planta_digital = c2d(planta, Ts, 'zoh');

%% Escalon de 0.1 m = 10 cm
referencia = zeros(size(tiempo));
referencia(tiempo >= instante_escalon) = 10;
posicion_continua = lsim(T, referencia, tiempo, 'zoh');
posicion_digital_lineal = lsim(Tz, referencia, tiempo);
[posicion_escalon, pulso_escalon, saturadas] = simular_controlador( ...
    planta_digital, b, a, referencia, c_simulacion, ...
    pulso_centro, pulso_min, pulso_max, 0, 0);

fig_escalon = figure('Name',[nombre_controlador ' - Escalon de 10 cm'],'Color','white');
subplot(2,1,1);
plot(tiempo, referencia, ':k', tiempo, posicion_continua, '--', ...
    tiempo, posicion_digital_lineal, '-.', tiempo, posicion_escalon);
grid on; ylabel('Posicion [cm]');
legend('Referencia','Continuo lineal','Digital lineal', ...
    'Digital con limites y redondeo','Location','best');
subplot(2,1,2);
stairs(tiempo, pulso_escalon); grid on; hold on;
yline(pulso_min, '--r', 'Minimo'); yline(pulso_max, '--r', 'Maximo');
ylabel('Pulso [us]'); xlabel('Tiempo [s]');

inicio = find(tiempo >= instante_escalon, 1);
info_digital = stepinfo(posicion_escalon(inicio:end), ...
    tiempo(inicio:end)-tiempo(inicio), 10);
final = tiempo >= tiempo(end)-2;
fprintf('\n--- Escalon de 10 cm: %s digital con limites y redondeo ---\n', nombre_controlador);
fprintf('Sesgo simulado: %.7g cm/s^2\n', c_simulacion);
fprintf('Establecimiento al 2 %% respecto de 10 cm: %.4f s\n', info_digital.SettlingTime);
fprintf('Sobrepaso: %.2f %%\n', info_digital.Overshoot);
fprintf('Error medio en los ultimos 2 s: %.5f cm\n', ...
    mean(referencia(final)-posicion_escalon(final)));
fprintf('Pulso minimo/maximo: %.0f / %.0f us; muestras saturadas: %d\n', ...
    min(pulso_escalon), max(pulso_escalon), saturadas);
fprintf('Error estacionario nominal lineal: %.5g cm\n', 10*(1-dcgain(Tz)));

%% Golpe con referencia fija en cero
% Un impulso mecanico cambia la velocidad del carrito. No usar impulse(T):
% eso seria un impulso de REFERENCIA. La fuerza real del golpe no se conoce,
% por lo que el salto de velocidad elegido es una amplitud ilustrativa.
referencia_golpe = zeros(size(tiempo));
muestra_golpe = find(tiempo >= instante_golpe, 1);
[posicion_golpe, pulso_golpe, saturadas_golpe] = simular_controlador( ...
    planta_digital, b, a, referencia_golpe, c_simulacion, ...
    pulso_centro, pulso_min, pulso_max, muestra_golpe, salto_velocidad);
fig_golpe = figure('Name',[nombre_controlador ' - Recuperacion ante un golpe'],'Color','white');
subplot(2,1,1);
plot(tiempo, referencia_golpe, ':k', tiempo, posicion_golpe); grid on;
xline(tiempo(muestra_golpe), '--', 'Golpe'); ylabel('Posicion [cm]');
legend('Referencia',[nombre_controlador ' digital'],'Location','best');
subplot(2,1,2);
stairs(tiempo, pulso_golpe); grid on; hold on;
yline(pulso_min, '--r', 'Minimo'); yline(pulso_max, '--r', 'Maximo');
ylabel('Pulso [us]'); xlabel('Tiempo [s]');
fprintf('\n--- Golpe: salto de velocidad %.2f cm/s en t=%.2f s ---\n', ...
    salto_velocidad, tiempo(muestra_golpe));
fprintf('Maximo desplazamiento absoluto despues del golpe: %.5f cm\n', ...
    max(abs(posicion_golpe(muestra_golpe:end))));
fprintf('Pulso minimo/maximo: %.0f / %.0f us; muestras saturadas: %d\n', ...
    min(pulso_golpe), max(pulso_golpe), saturadas_golpe);


%% Simulacion de la recurrencia Arduino cada Ts segundos
function [posicion, pulso, saturadas] = simular_controlador( ...
    planta, b, a, referencia, sesgo, centro, minimo, maximo, muestra_golpe, salto)
    estado = zeros(4,1); % Comenzar en posicion cero y en reposo
    posicion = zeros(size(referencia));
    pulso = zeros(size(referencia));
    saturadas = 0;
    errores = zeros(2,1);
    salidas = zeros(2,1);
    for k = 1:numel(referencia)
        if k == muestra_golpe
            estado(2) = estado(2)+salto;
        end
        posicion(k) = estado(1);
        error = referencia(k)-posicion(k);
        salida = -a(2:3)*salidas + b*[error; errores];
        pedido = centro+salida;
        if pedido < minimo || pedido > maximo
            saturadas = saturadas+1;
        end
        % Misma saturacion y redondeo a microsegundos que en Arduino.
        limitado = min(max(pedido, minimo), maximo);
        pulso(k) = round(limitado);
        errores = [error; errores(1)];
        % Igual que el sketch: memoria limitada, sin cuantizar la recurrencia.
        % Proteccion simple al saturar; no es anti-windup universal.
        salidas = [limitado-centro; salidas(1)];
        estado = planta.A*estado + planta.B*[pulso(k)-centro; sesgo];
    end
end
