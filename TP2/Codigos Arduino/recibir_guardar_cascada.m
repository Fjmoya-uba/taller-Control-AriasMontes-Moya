function datos = recibir_guardar_cascada(puerto, duracion_s, referencia_cm)
% Ejemplo: datos = recibir_guardar_cascada("COM3", 60);
% La referencia NO se transmite. El tercer argumento solo dibuja la referencia
% y debe coincidir con REFERENCIA_CM del sketch; no envia comandos al Arduino.
% Cerrar el Monitor Serie antes de ejecutar. Mantener quieta la barra
% durante la calibracion inicial. Requiere serialport (MATLAB R2019b+).
% Distancia -1 = sin eco; el sonar se actualiza cada 20 ms, TX cada 20 ms.
% Trama: abcd + servo_us + distancia_cm + referencia_angulo_deg + angulo_deg (20 bytes).
arguments
    puerto (1,1) string = "COM3"
    duracion_s (1,1) double {mustBePositive, mustBeFinite} = 30
    referencia_cm (1,1) double {mustBeFinite} = 17
end

conexion = serialport(puerto, 115200, "Timeout", 2);
limpieza = onCleanup(@() delete(conexion)); %#ok<NASGU>
flush(conexion);
cabecera = uint8('abcd');
buffer = zeros(1, 0, 'uint8');
mediciones = zeros(ceil(duracion_s / 0.02) + 100, 4, 'single');
tiempo = zeros(size(mediciones, 1), 1);
cantidad = 0;
[~, ~, endian] = computer;

figura = figure('Name', 'Control de posicion del carrito');
ax1 = subplot(3, 1, 1, 'Parent', figura);
hr = animatedline(ax1, 'Color', [0.85 0.33 0.10]);
hy = animatedline(ax1, 'Color', [0 0.45 0.74]);
grid(ax1, 'on'); ylabel(ax1, 'Distancia [cm]');
legend(ax1, 'Referencia r', 'Distancia', 'Location', 'best');
axError = subplot(3, 1, 2, 'Parent', figura);
har = animatedline(axError, 'Color', [0.85 0.33 0.10]);
he = animatedline(axError, 'Color', [0.49 0.18 0.56]);
grid(axError, 'on'); ylabel(axError, 'Angulo [grados]');
legend(axError, 'Referencia interna', 'Filtro complementario', 'Location', 'best');
yline(axError, 0, '--');
ax2 = subplot(3, 1, 3, 'Parent', figura);
hu = animatedline(ax2, 'Color', [0.47 0.67 0.19]);
grid(ax2, 'on'); ylabel(ax2, 'Entrada u [us]'); xlabel(ax2, 'Tiempo de recepcion [s]');



yline(ax2, 1000, '--', 'Minimo');
yline(ax2, 2000, '--', 'Maximo');
ylim(ax2, [950 2050]);
linkaxes([ax1 axError ax2], 'x');
estado = title(ax1, 'Esperando datos del filtro...');

disp('Esperando calibracion y primera trama...');
espera = tic;
reloj = [];
ultimaTrama = tic;
avisoSinDatos = false;
while isgraphics(figura)
    if isempty(reloj)
        if toc(espera) > 15
            error('No llegaron tramas en 15 s. Revisar puerto, sketch e IMU.');
        end
    elseif toc(reloj) >= duracion_s
        break;
    end
    if ~isempty(reloj) && toc(ultimaTrama) > 2 && ~avisoSinDatos
        warning('Carrito:SinDatos', ...
            'Sin tramas validas durante 2 s. Revisar Arduino, IMU y conexion USB.');
        estado.String = 'Sin datos desde hace mas de 2 s';
        avisoSinDatos = true;
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
            buffer = buffer(max(1, end-2):end); % Cabecera parcial.
            break;
        end
        buffer = buffer(inicios(1):end);
        if numel(buffer) < 20
            break; % Esperar el resto, conservando los bytes recibidos.
        end
        valores = typecast(buffer(5:20), 'single');
        buffer = buffer(21:end);
        if endian == 'B'
            valores = swapbytes(valores);
        end
        if any(~isfinite(valores))
            continue;
        end
        ultimaTrama = tic;
        avisoSinDatos = false;
        if isempty(reloj)
            reloj = tic;
            disp('Registrando y graficando...');
        end
        cantidad = cantidad + 1;
        if cantidad > size(mediciones, 1)
            mediciones(end+1000, 4) = single(0);
            tiempo(end+1000, 1) = 0;
        end
        tiempo(cantidad) = toc(reloj);
        mediciones(cantidad, :) = valores;
        addpoints(hr, tiempo(cantidad), referencia_cm);
        posicion = double(valores(2));
        if posicion < 0, posicion = NaN; end
        addpoints(hy, tiempo(cantidad), posicion);
        addpoints(hu, tiempo(cantidad), double(valores(1)));
        errorActual = referencia_cm - double(valores(2));
        if valores(2) < 0, errorActual = NaN; end
        addpoints(har, tiempo(cantidad), double(valores(3)));
        addpoints(he, tiempo(cantidad), double(valores(4)));
        estado.String = sprintf('r = %.2f cm | posicion = %.2f cm | error = %.2f cm | angulo = %.2f deg | PWM = %.0f us', ...
            referencia_cm, valores(2), errorActual, valores(4), valores(1));
    end
    drawnow limitrate;
end

tiempo = tiempo(1:cantidad);
mediciones = mediciones(1:cantidad, :);
datos = timetable(seconds(tiempo), mediciones(:,1), mediciones(:,2), ...
    mediciones(:,3), mediciones(:,4), 'VariableNames', ...
    {'servo_us', 'distancia_cm', 'referencia_angulo_deg', 'angulo_deg'});
datos.referencia_cm = repmat(single(referencia_cm), cantidad, 1); % Configurada en PC.
datos.error_angulo_deg = datos.referencia_angulo_deg - datos.angulo_deg;
datos.error_cm = datos.referencia_cm - datos.distancia_cm;
datos.error_cm(datos.distancia_cm < 0) = NaN;
archivoSalida = fullfile(fileparts(mfilename('fullpath')), ...
    ['mediciones_cascada_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS')) '.mat']);
save(archivoSalida, 'datos', 'tiempo', 'mediciones', 'referencia_cm');
fprintf('Se guardaron %d muestras en %s\n', cantidad, archivoSalida);
end

