%% Visualizar dos o tres mediciones en funcion del tiempo
% Ejecutar, elegir el archivo, la variable de tiempo y las senales.
% MAT: vectores guardados directamente o dentro de la estructura out.
% CSV/TXT: una columna de tiempo y una columna por cada senal.
% Grafica los valores originales, sin filtrar ni cambiar sus unidades.
clear; clc; close all;

%% 1. Elegir y leer el archivo
[archivo, carpeta] = uigetfile( ...
    {'*.mat;*.csv;*.txt', 'Mediciones (*.mat, *.csv, *.txt)'}, ...
    'Elegir archivo de mediciones');
if isequal(archivo, 0)
    return;
end
ruta = fullfile(carpeta, archivo);
[~, ~, extension] = fileparts(archivo);

if strcmpi(extension, '.mat')
    datos = load(ruta);
    % Los registros del carrito tienen out.tout, out.d1, out.d2, etc.
    if isfield(datos, 'out')
        datos = datos.out;
    end
    nombres = fieldnames(datos);
else
    tabla = readtable(ruta);
    datos = table2struct(tabla, 'ToScalar', true);
    nombres = fieldnames(datos);
end

%% 2. Mostrar solamente variables numericas que sean vectores
disponibles = {};
for k = 1:numel(nombres)
    valor = datos.(nombres{k});
    if isnumeric(valor) && isreal(valor) && isvector(valor) && numel(valor) > 1
        disponibles{end+1} = nombres{k};
    end
end
assert(numel(disponibles) >= 3, ...
    'Se necesitan un vector de tiempo y al menos dos senales numericas.');

%% 3. Elegir el tiempo y las dos o tres senales
% Preseleccionar tout si existe, como en los archivos del carrito.
opcion_tiempo = find(strcmp(disponibles, 'tout'), 1);
if isempty(opcion_tiempo)
    opcion_tiempo = 1;
end
[seleccion, aceptado] = listdlg('PromptString', 'Elegir la variable de tiempo:', ...
    'ListString', disponibles, 'SelectionMode', 'single', ...
    'InitialValue', opcion_tiempo, 'ListSize', [350 220]);
if ~aceptado
    return;
end
nombre_tiempo = disponibles{seleccion};
tiempo = double(datos.(nombre_tiempo));
tiempo = tiempo(:);
assert(all(isfinite(tiempo)) && all(diff(tiempo) > 0), ...
    'El tiempo debe contener valores finitos y estar ordenado de menor a mayor.');

% Ofrecer senales con la misma cantidad de muestras que el tiempo.
senales = {};
for k = 1:numel(disponibles)
    nombre = disponibles{k};
    if ~strcmp(nombre, nombre_tiempo) && numel(datos.(nombre)) == numel(tiempo)
        senales{end+1} = nombre;
    end
end
assert(numel(senales) >= 2, 'No hay dos senales con la misma longitud que el tiempo.');
[seleccion, aceptado] = listdlg( ...
    'PromptString', 'Elegir 2 o 3 senales (usar Ctrl para seleccionar):', ...
    'ListString', senales, 'SelectionMode', 'multiple', ...
    'InitialValue', 1:min(3, numel(senales)), 'ListSize', [400 240]);
if ~aceptado
    return;
end
assert(numel(seleccion) >= 2 && numel(seleccion) <= 3, ...
    'Elegir dos o tres senales.');

%% 4. Graficar cada senal por separado, con el mismo eje de tiempo
figure('Name', archivo, 'Color', 'white');
ejes = gobjects(numel(seleccion), 1);
for k = 1:numel(seleccion)
    nombre = senales{seleccion(k)};
    medicion = double(datos.(nombre));
    ejes(k) = subplot(numel(seleccion), 1, k);
    plot(tiempo, medicion(:), '.-', 'MarkerSize', 5);
    grid on;
    ylabel(nombre, 'Interpreter', 'none');
    title(nombre, 'Interpreter', 'none');
end
xlabel(['Tiempo (' nombre_tiempo ')'], 'Interpreter', 'none');
linkaxes(ejes, 'x'); % Al hacer zoom, todos muestran el mismo intervalo.
sgtitle(archivo, 'Interpreter', 'none');
fprintf('Archivo: %s\nMuestras: %d\nTiempo: %.3f a %.3f\n', ...
    archivo, numel(tiempo), tiempo(1), tiempo(end));
