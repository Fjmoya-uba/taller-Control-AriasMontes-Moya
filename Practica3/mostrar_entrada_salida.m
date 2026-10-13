function mostrar_entrada_salida(puerto, duracion_s)
% Ejemplo: mostrar_entrada_salida("COM3", 30)
% Cerrar el Monitor Serie y cargar filtro_complementario_mpu6050_v5_excitacion.
% Mantener el sistema quieto durante la calibracion inicial (~8 s).
% Arduino estima el offset en 1544 us y envia el angulo relativo a ese cero.
% Trama: 'abcd' + salida medida [grados] + orden al servo [us], float32 LE.
% La entrada es el valor enviado a writeMicroseconds, no un pulso medido.
arguments
    puerto (1,1) string = "COM7"
    duracion_s (1,1) double {mustBePositive, mustBeFinite} = 30
end

conexion = serialport(puerto, 115200, "Timeout", 2);
limpieza = onCleanup(@() delete(conexion)); %#ok<NASGU>
flush(conexion);
buffer = zeros(1, 0, 'uint8');
cabecera = uint8('abcd');
[~, ~, endian] = computer;

figura = figure('Name', 'Entrada y salida del servo');
axY = subplot(2, 1, 1, 'Parent', figura);
hy = animatedline(axY, 'Color', [0 0.45 0.74]);
grid(axY, 'on'); ylabel(axY, 'Salida y [grados]');
title(axY, 'Angulo relativo al cero calibrado en 1544 us');
axU = subplot(2, 1, 2, 'Parent', figura);
hu = animatedline(axU, 'Color', [0.85 0.33 0.10]);
grid(axU, 'on'); ylabel(axU, 'Entrada u [us]');
xlabel(axU, 'Tiempo de recepcion [s]');
title(axU, 'Orden enviada al servo');
linkaxes([axY axU], 'x');

disp('Esperando calibracion del cero en 1544 us y primera trama...');
espera = tic;
reloj = [];
fprintf('Tiempo [s]\tSalida [grados]\tEntrada ordenada [us]\n');
while isgraphics(figura)
    if isempty(reloj)
        if toc(espera) > 20
            error('No llegaron tramas en 20 s. Revisar puerto, sketch e IMU.');
        end
    elseif toc(reloj) >= duracion_s
        break;
    end
    disponibles = conexion.NumBytesAvailable;
    if disponibles == 0
        drawnow limitrate;
        pause(0.005);
        continue;
    end
    buffer = [buffer, reshape(read(conexion, disponibles, 'uint8'), 1, [])]; %#ok<AGROW>
    while numel(buffer) >= 4
        inicios = strfind(buffer, cabecera);
        if isempty(inicios)
            buffer = buffer(max(1, end-2):end);
            break;
        end
        buffer = buffer(inicios(1):end);
        if numel(buffer) < 12
            break;
        end
        valores = typecast(buffer(5:12), 'single');
        buffer = buffer(13:end);
        if endian == 'B'
            valores = swapbytes(valores);
        end
        if any(~isfinite(valores))
            continue;
        end
        if isempty(reloj)
            reloj = tic;
        end
        t = toc(reloj);
        fprintf('%.3f\t\t%.3f\t\t%.0f\n', t, valores(1), valores(2));
        addpoints(hy, t, double(valores(1)));
        addpoints(hu, t, double(valores(2)));
    end
    drawnow limitrate;
end
end
