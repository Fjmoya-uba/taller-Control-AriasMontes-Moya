function exportar_sesgo_tustin
% Rutas relativas a este archivo, independientemente del Current Folder.
carpeta = fileparts(mfilename('fullpath'));
[Ad, Bu, By, Ts] = calcular_observador(carpeta);
assert(all(abs(eig(Ad)) < 1), 'Observador inestable');
destino = fullfile(carpeta, 'validar_sesgo_tustin');
if ~isfolder(destino)
    [ok, mensaje] = mkdir(destino);
    assert(ok, 'No se pudo crear la carpeta %s: %s', destino, mensaje);
end
ruta = fullfile(destino, 'coeficientes_sesgo_tustin.h');
[fid, mensaje] = fopen(ruta, 'w');
assert(fid >= 0, 'No se pudo escribir %s: %s', ruta, mensaje);
limpieza = onCleanup(@() fclose(fid));
fprintf(fid, '// Generado por exportar_sesgo_tustin.m. Grados, grados/s, us.\n#pragma once\n');
nombres = {'OBS_AD', 'OBS_BU', 'OBS_BY'};
matrices = {Ad, Bu, By};
for k = 1:3
    V = matrices{k};
    if size(V,2) == 1
        fprintf(fid, 'const float %s[%d] = {', nombres{k}, size(V,1));
        fprintf(fid, '%.12ef, ', V);
        fprintf(fid, '};\n');
    else
        fprintf(fid, 'const float %s[%d][%d] = {\n', nombres{k}, size(V,1), size(V,2));
        for fila = 1:size(V,1)
            fprintf(fid, '  {');
            fprintf(fid, '%.12ef, ', V(fila,:));
            fprintf(fid, '},\n');
        end
        fprintf(fid, '};\n');
    end
end
% Mismo filtro complementario Tustin que en el observador comprobado.
tau = -Ts/log(0.94);
fprintf(fid, 'const float FILTRO_A = %.12ef;\n', (2*tau-Ts)/(2*tau+Ts));
fprintf(fid, 'const float FILTRO_ACC = %.12ef;\n', Ts/(2*tau+Ts));
fprintf(fid, 'const float FILTRO_GYRO = %.12ef;\n', tau*Ts/(2*tau+Ts));
clear limpieza;
fprintf('Generado: %s\n', ruta);
end

function [Ad, Bu, By, Ts] = calcular_observador(carpeta)
% Aislar el clear del script para que no borre las rutas del exportador.
run(fullfile(carpeta, 'sumamos_el_sesgo.m'));
end
