%% Visualizar multiples senales agrupadas en distintos subplots
% Ejecutar, elegir el archivo, la variable de tiempo y los grupos de senales.
% MAT: vectores guardados directamente o dentro de la estructura out.
% CSV/TXT: una columna de tiempo y una columna por cada senal.
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
    if isfield(datos, 'out')
        datos = datos.out;
    end
    nombres = fieldnames(datos);
else
    tabla = readtable(ruta);
    datos = table2struct(tabla, 'ToScalar', true);
    nombres = fieldnames(datos);
end

%% 2. Filtrar variables numericas que sean vectores
disponibles = {};
for k = 1:numel(nombres)
    valor = datos.(nombres{k});
    if isnumeric(valor) && isreal(valor) && isvector(valor) && numel(valor) > 1
        disponibles{end+1} = nombres{k};
    end
end
assert(numel(disponibles) >= 2, ...
    'Se necesitan al menos un vector de tiempo y una senal numerica.');

%% 3. Elegir la variable de tiempo
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

% Filtrar senales con la misma cantidad de muestras que el tiempo
senales = {};
for k = 1:numel(disponibles)
    nombre = disponibles{k};
    if ~strcmp(nombre, nombre_tiempo) && numel(datos.(nombre)) == numel(tiempo)
        senales{end+1} = nombre;
    end
end
assert(~isempty(senales), 'No hay senales con la misma longitud que el tiempo.');

%% 4. Configurar cantidad de subplots y senales por cada uno
respuesta = inputdlg({'Cantidad de graficos apilados (filas):'}, ...
    'Configuracion de Layout', [1 40], {'2'});
if isempty(respuesta)
    return;
end
num_graficos = round(str2double(respuesta{1}));
assert(~isnan(num_graficos) && num_graficos >= 1 && num_graficos <= 10, ...
    'Ingrese un numero valido de graficos entre 1 y 10.');

grupos_senales = cell(num_graficos, 1);
for g = 1:num_graficos
    prompt = sprintf('Grafico %d de %d: Elegir senales (Ctrl/Shift para varias):', g, num_graficos);
    [sel_g, aceptado] = listdlg( ...
        'PromptString', prompt, ...
        'ListString', senales, 'SelectionMode', 'multiple', ...
        'InitialValue', min(g, numel(senales)), 'ListSize', [400 240]);
    if ~aceptado || isempty(sel_g)
        return;
    end
    grupos_senales{g} = senales(sel_g);
end

%% 5. Graficar cada grupo en su propio subplot con eje X sincronizado
figure('Name', archivo, 'Color', 'white');
ejes = gobjects(num_graficos, 1);

for g = 1:num_graficos
    ejes(g) = subplot(num_graficos, 1, g);
    hold on; % Permite superponer multiples senales en el mismo subplot
    
    lista_actual = grupos_senales{g};
    for s = 1:numel(lista_actual)
        nombre = lista_actual{s};
        medicion = double(datos.(nombre));
        plot(tiempo, medicion(:), 'MarkerSize', 5, 'DisplayName', nombre);
    end
    
    hold off;
    grid on;
    % Mostrar leyenda automaticamente para distinguir las curvas del subplot
    legend('show', 'Interpreter', 'none', 'Location', 'best');
    
    % Si hay una sola senal, usarla de etiqueta Y; si hay varias, dejar generico
    if numel(lista_actual) == 1
        ylabel(lista_actual{1}, 'Interpreter', 'none');
    else
        ylabel(sprintf('Grupo %d', g));
    end
end

xlabel(['Tiempo (' nombre_tiempo ')'], 'Interpreter', 'none');
linkaxes(ejes, 'x'); % Mantiene el zoom sincronizado en todos los graficos
sgtitle(archivo, 'Interpreter', 'none');

fprintf('Archivo: %s | Muestras: %d | Tiempo: %.3f a %.3f s\n', ...
    archivo, numel(tiempo), tiempo(1), tiempo(end));