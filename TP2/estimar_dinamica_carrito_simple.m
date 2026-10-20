%% Identificacion del carrito a partir de su posicion
% Modelo: posicion'' = K * angulo - a * posicion' + c
% Ajuste con posicion inicial medida y velocidad inicial fija en cero.
% Requiere Control System Toolbox (ss, lsim y tf).
clear; clc; close all;

%% 1. Leer las mediciones
datos = load('mediciones_dinamica_carrito_1.mat');
tiempo = double(datos.out.tout(:));
angulo = double(datos.out.d1(:));             % Grados, cero horizontal
posicion = double(datos.out.d2(:)) - 15.612;  % Centimetros


posicion_inicial = 15.9115 - 15.612; % Distancia del sensor -> posicion [cm]
velocidad_inicial = 0;               % Carrito inicialmente en reposo

%% 2. Buscar el bloque mas largo de mediciones validas consecutivas
valida = isfinite(angulo) & isfinite(posicion) & ...
    posicion >= -15 & posicion <= 21 & tiempo <= 34.66;

% Recorrer las muestras sin pegar bloques separados por datos invalidos.
inicio_actual = 1;
inicio_mejor = 1;
cantidad_mejor = 0;
for muestra = 1:numel(tiempo)
    if ~valida(muestra)
        inicio_actual = muestra + 1;
    else
        cantidad_actual = muestra - inicio_actual + 1;
        if cantidad_actual > cantidad_mejor
            inicio_mejor = inicio_actual;
            cantidad_mejor = cantidad_actual;
        end
    end
end
assert(cantidad_mejor >= 100, 'No hay suficientes muestras validas.');
assert(inicio_mejor == 1 && abs(tiempo(1)) < 1e-9, ...
    'El bloque seleccionado no empieza en t=0: revisar datos antes de fijar condiciones iniciales.');
filas = inicio_mejor:(inicio_mejor + cantidad_mejor - 1);
fprintf('Tramo continuo: filas %d a %d, tiempos %.3f a %.3f s.\n', ...
    filas(1), filas(end), tiempo(filas(1)), tiempo(filas(end)));
fprintf('Se excluyeron %d muestras.\n', numel(tiempo)-numel(filas));
tiempo = tiempo(filas) - tiempo(filas(1));
angulo = angulo(filas);
posicion = posicion(filas);
periodo = median(diff(tiempo));

fprintf('Ts = %.6f s (%.2f Hz).\n', periodo, 1/periodo);

%% 3. Separar entrenamiento (primer 75 por ciento) y validacion
cantidad_estimacion = floor(0.75*numel(tiempo));
entrenamiento = 1:cantidad_estimacion;
validacion = (cantidad_estimacion+1):numel(tiempo);
tiempo_est = tiempo(entrenamiento);
angulo_est = angulo(entrenamiento);
posicion_est = posicion(entrenamiento);

%% 4. Buscar el amortiguamiento que da menor error de posicion
% Para cada a se ajustan solamente K y c; las condiciones iniciales son fijas.
amortiguamientos = [0, logspace(-4, 2, 100)];
errores = zeros(size(amortiguamientos));
for prueba = 1:numel(amortiguamientos)
    errores(prueba) = ajustar(amortiguamientos(prueba), ...
        tiempo_est, angulo_est, posicion_est, posicion_inicial);
end
[~, mejor] = min(errores);
a = amortiguamientos(mejor);
if mejor > 1 && mejor < numel(amortiguamientos)
    funcion_error = @(valor) ajustar(valor, tiempo_est, angulo_est, posicion_est, posicion_inicial);
    a_refinado = fminbnd(funcion_error, ...
        amortiguamientos(mejor-1), amortiguamientos(mejor+1));
    if funcion_error(a_refinado) < errores(mejor)
        a = a_refinado;
    end
end
[~, parametros] = ajustar(a, tiempo_est, angulo_est, posicion_est, posicion_inicial);
K = parametros(1);
c = parametros(2);

%% 5. Simular todo el tramo sin reiniciar al llegar a validacion
posicion_simulada = posicion_inicial + respuestas(a, tiempo, angulo)*parametros;
[~, parametros_doble] = ajustar(0, tiempo_est, angulo_est, posicion_est, posicion_inicial);
posicion_doble = posicion_inicial + respuestas(0, tiempo, angulo)*parametros_doble;
[rmse, fit] = metricas(posicion(validacion), posicion_simulada(validacion));
[rmse_doble, fit_doble] = metricas(posicion(validacion), posicion_doble(validacion));

fprintf('\nK = %.8g cm/(grado*s^2), a = %.8g s^-1\n', K, a);
fprintf('c = %.8g cm/s^2, y inicial = %.6g cm, v inicial = %.6g cm/s\n', ...
    c, posicion_inicial, velocidad_inicial);
fprintf('G(s) = %.8g / [s*(s+%.8g)]\n', K, a);
fprintf('Estimacion: %d muestras. Validacion: %d muestras.\n', ...
    numel(entrenamiento), numel(validacion));
fprintf('Viscoso: RMSE = %.6g cm | FIT = %.2f %%\n', rmse, fit);
fprintf('Doble integrador: RMSE = %.6g cm | FIT = %.2f %%\n', rmse_doble, fit_doble);
if a >= 99
    warning('El amortiguamiento alcanzo el limite de busqueda.');
end
if K <= 0
    warning('Ganancia no positiva: revisar signos y calidad del ajuste.');
end
G_carrito = tf(K, [1 a 0]); % No incluye c ni las condiciones iniciales

figure('Name', 'Identificacion del carrito - version simple', 'Color', 'white');
subplot(3,1,1);
plot(tiempo, angulo); grid on; ylabel('Angulo [grados]');
xline(tiempo(validacion(1)), ':k', 'Validacion');
subplot(3,1,2);
plot(tiempo, posicion, tiempo, posicion_simulada, '--', tiempo, posicion_doble, ':');
grid on; ylabel('Posicion [cm]');
legend('Medida', 'Viscoso', 'Doble integrador', 'Location', 'best');
xline(tiempo(validacion(1)), ':k');
subplot(3,1,3);
plot(tiempo(validacion), posicion(validacion)-posicion_simulada(validacion));
grid on; ylabel('Error [cm]'); xlabel('Tiempo desde inicio del tramo [s]');

%% Funciones auxiliares
function [error, parametros] = ajustar(a, tiempo, angulo, posicion, posicion_inicial)
    matriz = respuestas(a, tiempo, angulo);
    % La barra invertida resuelve minimos cuadrados: matriz*parametros ~= posicion.
    parametros = matriz \ (posicion-posicion_inicial);
    posicion_simulada = posicion_inicial + matriz*parametros;
    error = mean((posicion_simulada-posicion).^2);
end

function matriz = respuestas(a, tiempo, angulo)
    % Estados: [posicion; velocidad]. Entrada: aceleracion aplicada.
    sistema = ss([0 1; 0 -a], [0; 1], [1 0], 0);
    % Con velocidad inicial cero: posicion = posicion_inicial
    % + K*respuesta_angulo + c*respuesta_constante.
    respuesta_angulo = lsim(sistema, angulo, tiempo, [0; 0], 'zoh');
    respuesta_constante = lsim(sistema, ones(size(tiempo)), tiempo, [0; 0], 'zoh');
    matriz = [respuesta_angulo, respuesta_constante];
end

function [rmse, fit] = metricas(posicion, simulada)
    error = posicion-simulada;
    rmse = sqrt(mean(error.^2));
    referencia = norm(posicion-mean(posicion));
    fit = NaN;
    if referencia > eps
        fit = 100*(1-norm(error)/referencia);
    end
end
