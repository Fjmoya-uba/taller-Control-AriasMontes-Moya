%% Guardar los diez canales del receptor de Simulink
% Requiere out con d1...d10; cada canal debe ser un vector numerico.
% d1 es el angulo medido; out.tout es el tiempo de Simulink.
nombres = {'theta_medida', 'theta_estimada', ...
    'omega_medida', 'omega_estimada', 'posicion_medida', ...
    'posicion_estimada', 'velocidad_ventana', 'velocidad_estimada', ...
    'u_aplicado', 'u_pedido'};
registro = struct();
registro.tiempo = out.tout(:);
for i = 1:numel(nombres)
    canal = out.(sprintf('d%d', i));
    assert(isnumeric(canal) && isvector(canal), ...
        'Configurar las salidas del receptor como vectores numericos.');
    registro.(nombres{i}) = canal(:);
end
[archivo, carpeta] = uiputfile('*.mat', 'Guardar ensayo', ...
    'control_sin_referencia_ensayo_01.mat');
if ~isequal(archivo, 0)
    save(fullfile(carpeta, archivo), 'registro');
end
