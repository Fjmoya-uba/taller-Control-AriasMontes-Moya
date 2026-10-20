%% Comparar la planta completa sin retardo y con retardo fijo
% Entrada: comando del servo (d3). Salidas medidas: angulo (d1), distancia (d2).
% No se ajustan parametros: solamente se simulan los modelos ya obtenidos.
% Requiere Control System Toolbox. No utiliza Pade.
clear; clc; close all;

%% 1. Opciones y parametros del modelo
retardos = [0, 0.02, 0.04]; % Segundos; deben ser multiplos del periodo medido
tiempo_final = 34.64;      % Mismo final de tramo que en la identificacion
K = 19.619739;
a = 5.7317425;
c = -3.2630965;
posicion_inicial = 0.2995; % cm, distancia inicial 15.9115 menos referencia 15.612
velocidad_inicial = 0;

%% 2. Cargar el registro, conservando su inicio en t=0
datos = load('mediciones_dinamica_carrito_2.mat');
tiempo = double(datos.out.tout(:));
angulo = double(datos.out.d1(:));
posicion = double(datos.out.d2(:)) - 15.612;
comando = double(datos.out.d3(:)) - 1544; % Variacion del pulso [us]
assert(numel(tiempo)==numel(angulo) && numel(tiempo)==numel(posicion) && ...
    numel(tiempo)==numel(comando), 'Los vectores tienen longitudes distintas.');
assert(all(isfinite(tiempo)) && all(diff(tiempo)>0) && abs(tiempo(1))<1e-9, ...
    'El tiempo debe empezar en cero y ser finito y creciente.');
% Cortar antes del primer dato invalido; no unir segmentos separados.
valida = isfinite(angulo) & isfinite(posicion) & isfinite(comando) & ...
    posicion >= -15 & posicion <= 21 & tiempo <= tiempo_final + 1e-9;
primera_invalida = find(~valida, 1);
if isempty(primera_invalida)
    ultima = numel(tiempo);
else
    ultima = primera_invalida-1;
end
assert(ultima >= 100, 'No hay suficientes datos validos desde t=0.');
tiempo = tiempo(1:ultima);
angulo = angulo(1:ultima);
posicion = posicion(1:ultima);
comando = comando(1:ultima);
periodo = median(diff(tiempo));
assert(max(abs(diff(tiempo)-periodo)) < 1e-6*periodo, ...
    'Se requiere muestreo uniforme, sin huecos.');
fprintf('Comparacion: %d muestras, de %.3f a %.3f s.\n', ...
    ultima, tiempo(1), tiempo(end));

%% 3. Conectar servo y carrito en un unico modelo
% Estados: posicion, velocidad, angulo y velocidad angular.
% Entradas: comando y una constante igual a 1 para incorporar c.
% Salidas: posicion y angulo. lsim integra ambos subsistemas juntos.
A = [0 1 0 0; 0 -a K 0; 0 0 0 1; 0 0 -2310 -91.07];
B = [0 0; 0 c; 0 0; 87.92 0];
C = [1 0 0 0; 0 0 1 0];
planta = ss(A, B, C, zeros(2,2));
% Misma condicion inicial para todas las opciones; servo inicialmente quieto.
estado_inicial = [posicion_inicial; velocidad_inicial; angulo(1); 0];

%% 4. Simular cada opcion con el mismo comando
angulos_simulados = zeros(numel(tiempo), numel(retardos));
posiciones_simuladas = zeros(numel(tiempo), numel(retardos));
rmse = zeros(numel(retardos), 1);
fit = zeros(numel(retardos), 1);
etiquetas = cell(1, numel(retardos));
% Mismo ultimo 25 por ciento reservado en la identificacion del carrito.
validacion = (floor(0.75*numel(tiempo))+1):numel(tiempo);
fprintf('Metricas sobre las ultimas %d muestras (sin reajustar modelos).\n', ...
    numel(validacion));
for opcion = 1:numel(retardos)
    muestras_retardo = round(retardos(opcion)/periodo);
    assert(retardos(opcion)>=0 && ...
        abs(muestras_retardo*periodo-retardos(opcion)) < 1e-9 && ...
        muestras_retardo < numel(tiempo), ...
        'Usar retardos no negativos, multiplos de Ts y menores que el registro.');
    % Retardo exacto en la grilla de muestras. Antes de t=0: comando horizontal.
    comando_retardado = [zeros(muestras_retardo,1); ...
        comando(1:end-muestras_retardo)];
    salidas = lsim(planta, [comando_retardado, ones(size(tiempo))], ...
        tiempo, estado_inicial, 'zoh');
    posicion_simulada = salidas(:,1);
    angulo_simulado = salidas(:,2);
    angulos_simulados(:,opcion) = angulo_simulado;
    posiciones_simuladas(:,opcion) = posicion_simulada;
    error = posicion(validacion)-posicion_simulada(validacion);
    rmse(opcion) = sqrt(mean(error.^2));
    referencia = norm(posicion(validacion)-mean(posicion(validacion)));
    fit(opcion) = NaN;
    if referencia > eps
        fit(opcion) = 100*(1-norm(error)/referencia);
    end
    etiquetas{opcion} = sprintf('Retardo %.3f s', retardos(opcion));
    fprintf('%s: RMSE = %.5f cm | FIT = %.2f %%\n', ...
        etiquetas{opcion}, rmse(opcion), fit(opcion));
end
resultados = table(retardos(:), rmse, fit, ...
    'VariableNames', {'Retardo_s', 'RMSE_cm', 'FIT_porcentaje'});
[~, mejor] = min(rmse);
fprintf('Menor RMSE entre las opciones: retardo %.3f s.\n', retardos(mejor));
fprintf('Es una comparacion sobre la misma corrida, no un ensayo independiente.\n');

%% 5. Comparar las curvas
figure('Name', 'Comparacion de retardo - planta completa', 'Color', 'white');
subplot(3,1,1);
stairs(tiempo, comando); grid on;
ylabel('Comando [us]'); title('Entrada respecto de 1544 us');
subplot(3,1,2);
plot(tiempo, angulo, 'k', tiempo, angulos_simulados, '--'); grid on;
ylabel('Angulo [grados]'); legend([{'Medido'}, etiquetas], 'Location', 'best');
subplot(3,1,3);
plot(tiempo, posicion, 'k', tiempo, posiciones_simuladas, '--'); grid on;
ylabel('Posicion [cm]'); xlabel('Tiempo [s]');
legend([{'Medida'}, etiquetas], 'Location', 'best');
xline(tiempo(validacion(1)), ':k', 'Validacion');
