function datos = recibir_observador(puerto, duracion_s)
% recibir_observador("COM7", 30): grafica y guarda las cuatro senales.
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

figura = figure('Name', 'Validacion del observador');
axY = subplot(2, 1, 1, 'Parent', figura);
hy = animatedline(axY, 'Color', [0 0.45 0.74]);
hyEst = animatedline(axY, 'Color', [0.85 0.33 0.10]);
grid(axY, 'on'); ylabel(axY, 'Angulo [rad]');
legend(axY, [hy hyEst], {'Filtro complementario', 'Observador'});
title(axY, 'Angulo relativo al centro');
axU = subplot(2, 1, 2, 'Parent', figura);
hu = animatedline(axU, 'Color', [0.85 0.33 0.10]);
huGyro = animatedline(axU, 'Color', [0 0.45 0.74]);
grid(axU, 'on'); ylabel(axU, 'Velocidad [rad/s]');
legend(axU, [hu huGyro], {'Observador', 'Giroscopo'});
xlabel(axU, 'Tiempo nominal [s]');
title(axU, 'Velocidad angular');
linkaxes([axY axU], 'x');

disp('Esperando calibracion del cero en 1544 us y primera trama...');
espera = tic;
reloj = [];
muestras = zeros(0, 5);
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
        if numel(buffer) < 20
            break;
        end
        valores = typecast(buffer(5:20), 'single');
        buffer = buffer(21:end);
        if endian == 'B'
            valores = swapbytes(valores);
        end
        if any(~isfinite(valores))
            continue;
        end
        if isempty(reloj)
            reloj = tic;
        end
        t = size(muestras, 1)*0.02;
        muestras(end+1, :) = [t double(valores)]; %#ok<AGROW>
        addpoints(hy, t, double(valores(4)));
        addpoints(hyEst, t, double(valores(3)));
        addpoints(hu, t, double(valores(1)));
        addpoints(huGyro, t, double(valores(2)));
    end
    drawnow limitrate;
end
datos = array2table(muestras, 'VariableNames', ...
    {'tiempo_s','omega_est_rad_s','omega_gyro_rad_s','theta_est_rad','theta_filtro_rad'});
% Tiempo reconstruido a 20 ms: solo valido si no hubo tramas/plazos perdidos.
carpeta = fileparts(mfilename('fullpath'));
save(fullfile(carpeta, 'mediciones_observador.mat'), 'datos');
end
