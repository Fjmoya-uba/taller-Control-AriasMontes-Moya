%% Recepcion y guardado de mediciones del MPU6050
% Compatible con IMU_Simulink_100Hz.ino.
% La trama es: "abcd" seguido por 6 valores single de 4 bytes cada uno.

clear;
clc;

puerto = "COM3";       % Cambiar por el puerto del Arduino
baudrate = 115200;
duracion_s = 30;        % Tiempo total que se desea registrar
archivoSalida = "mediciones_imu.mat";

serial = serialport(puerto, baudrate, "Timeout", 2);
flush(serial);           % Descarta bytes viejos que hayan quedado en el buffer

cabecera = uint8('abcd');
periodoEstimado = 0.01;  % TX_DECIMATION=1: una trama cada 10 ms
cantidadEstimada = ceil(duracion_s / periodoEstimado) + 100;

tiempo = zeros(cantidadEstimada, 1);
mediciones = zeros(cantidadEstimada, 6, 'single');
cantidad = 0;
reloj = tic;

disp("Registrando datos...");

while toc(reloj) < duracion_s
    % Ventana deslizante para encontrar el comienzo exacto de una trama.
    ventana = zeros(1, 4, 'uint8');
    while toc(reloj) < duracion_s
        ventana = [ventana(2:4), read(serial, 1, "uint8")]; %#ok<AGROW>
        if isequal(ventana, cabecera)
            break;
        end
    end

    if toc(reloj) >= duracion_s
        break;
    end

    payload = read(serial, 24, "uint8");

    % Arduino y las PC habituales almacenan los float en little-endian.
    valores = typecast(uint8(payload), 'single');

    cantidad = cantidad + 1;
    if cantidad > size(mediciones, 1)
        mediciones(end + 1000, 6) = single(0);
        tiempo(end + 1000, 1) = 0;
    end

    tiempo(cantidad) = toc(reloj);
    mediciones(cantidad, :) = valores;
end

clear serial;  % Libera el puerto COM

tiempo = tiempo(1:cantidad);
mediciones = mediciones(1:cantidad, :);

datos = timetable(seconds(tiempo), ...
    mediciones(:, 1), mediciones(:, 2), mediciones(:, 3), ...
    mediciones(:, 4), mediciones(:, 5), mediciones(:, 6), ...
    'VariableNames', {'ax', 'ay', 'az', 'gx', 'gy', 'gz'});

save(archivoSalida, 'datos', 'tiempo', 'mediciones');

fprintf("Se guardaron %d muestras en %s\n", cantidad, archivoSalida);

%% Ejemplo de visualizacion posterior
figure;
stackedplot(datos);
title("Mediciones del MPU6050");
