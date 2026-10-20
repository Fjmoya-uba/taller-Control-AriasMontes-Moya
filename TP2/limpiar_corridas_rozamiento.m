%% Recorta las 12 inclinaciones de ESTA captura, sin modificar el original.
% Ejecutar desde cualquier carpeta. No estima coeficientes de rozamiento.
clear; clc;
carpeta = fileparts(mfilename('fullpath'));
fuente = fullfile(carpeta,'mediciones_dinamica_carrito_1.mat');
S = load(fuente); t = double(S.out.tout(:));
angulo = double(S.out.d1(:)); cruda = double(S.out.d2(:));
Ts = median(diff(t));
valida = isfinite(cruda) & cruda>=2 & cruda<=40;
x = cruda; x(~valida)=NaN;
% Quitar solamente picos aislados; no interpolar huecos ni suavizar la salida.
mediana = movmedian(x,5,'omitnan');
pico = valida & abs(x-mediana)>1.5;
x(pico)=NaN;
posicionDeteccion=movmedian(x,9,'omitnan'); % Solo para detectar reposo.
% La captura muestra 12 inclinaciones separadas ~6 s, con ~4 s inclinada.
% El firmware actual (4+2+4 s) NO coincide: no asignar sus fases por indice.
% Buscar cada cambio real de angulo cerca de esos tiempos observados.
nominal = 2 + (0:11)*6;
cambio = abs(angulo-[repmat(angulo(1),5,1); angulo(1:end-5)]);
corridas = struct([]); todo = table();
resumen = table();
fig = figure('Visible','off','Position',[50 50 1500 1000]);
for r=1:12
    candidatos = find(t>=nominal(r)-0.5 & t<=nominal(r)+0.5 & cambio>1);
    assert(~isempty(candidatos),'No se encontro la inclinacion %d.',r);
    i0=candidatos(1); inicio=t(i0);
    % El regreso a horizontal se busca cerca de 4 s despues de inclinar.
    candidatos=find(t>inicio+3.4 & t<inicio+4.6 & cambio>1);
    if isempty(candidatos), limite=find(t<=inicio+4,1,'last');
    else, limite=candidatos(1)-1; end
    % Detectar desplazamiento sostenido en 0.12 s (>=0.6 cm).
    salto=6; inicioMov=[];
    for k=i0:min(limite-salto-2,numel(t)-salto-2)
        d=x(k+salto:k+salto+2)-x(k:k+2);
        if all(isfinite(d)) && all(abs(d)>0.6) && abs(sum(sign(d)))==3
            inicioMov=k; break;
        end
    end
    motivo="sin movimiento detectable"; finMov=[];
    if ~isempty(inicioMov)
        finMov=limite; motivo="fin de inclinacion/captura";
        for k=inicioMov:limite
            if ~isfinite(x(k))
                finMov=k-1;
                if ~valida(k), motivo="primer eco invalido";
                else, motivo="primer pico aislado"; end
                break;
            end
            % Reposo: 0.30 s con recorrido <=0.45 cm, tras avanzar >=2 cm.
            if k>=inicioMov+15 && k+15<=limite && ...
                    abs(x(k)-x(inicioMov))>=2
                z=posicionDeteccion(k:k+15);
                if all(isfinite(x(k:k+15))) && max(z)-min(z)<=0.6
                    finMov=k; motivo="posicion estable (posible contacto)"; break;
                end
            end
        end
    end
    if isempty(inicioMov) || finMov<inicioMov
        primero=i0; ultimo=min(limite,i0+25); util=false(size(t));
        duracion=0;
    else
        primero=max(1,inicioMov-8); ultimo=min(limite,finMov+8);
        util=(1:numel(t))'>=inicioMov & (1:numel(t))'<=finMov & isfinite(x);
        duracion=t(finMov)-t(inicioMov);
    end
    indices=(primero:ultimo)';
    if isempty(inicioMov), referencia=t(i0); else, referencia=t(inicioMov); end
    D=table(repmat(r,numel(indices),1),indices,t(indices),t(indices)-referencia,...
        angulo(indices),cruda(indices),x(indices),valida(indices),pico(indices),util(indices),...
        'VariableNames',{'corrida','indice_original','tiempo_original_s',...
        'tiempo_desde_movimiento_s','angulo_deg','distancia_cruda_cm',...
        'distancia_cm','eco_valido','pico_descartado','usar_ajuste'});
    corridas(r).datos=D;
    corridas(r).motivo_fin=motivo;
    corridas(r).inicio_inclinacion_s=inicio;
    corridas(r).duracion_util_s=duracion;
    fila=table(r,inicio,t(primero),t(ultimo),duracion,sum(D.usar_ajuste),...
        median(D.angulo_deg,'omitnan'),motivo,...
        'VariableNames',{'corrida','inclinacion_s','recorte_desde_s',...
        'recorte_hasta_s','duracion_util_s','muestras_utiles','angulo_mediano_deg','motivo_fin'});
    resumen=[resumen;fila]; %#ok<AGROW>
    todo=[todo;D]; %#ok<AGROW>
    subplot(4,3,r); plot(D.tiempo_original_s,D.distancia_cruda_cm,'.','Color',[.75 .75 .75]); hold on;
    plot(D.tiempo_original_s,D.distancia_cm,'b.-');
    plot(D.tiempo_original_s(D.usar_ajuste),D.distancia_cm(D.usar_ajuste),'g.');
    ylim([0 42]); grid on; title(sprintf('Corrida %d | %.2f s utiles',r,duracion));
    xlabel('Tiempo original [s]'); ylabel('Distancia [cm]');
end
notas = ["Recortes automaticos candidatos; no certifican movimiento libre ni ausencia de contacto."
    "Verde/usar_ajuste: movimiento hasta primer invalido, pico o reposo detectado."
    "Margen de 0.16 s antes/despues; el margen final no supera el fin de inclinacion."
    "No se interpola ni se deriva la posicion; los invalidos/picos son NaN."
    "d3 se excluye: es la cabecera abcd interpretada como float."
    "Firmware actual: ciclos de 10 s; captura observada: 6 s. Se alinea por cambios de angulo."
    "No concatenar corridas como si fueran una trayectoria continua."
    "No se corrige el cero de la IMU ni se resta el angulo medio."];
save(fullfile(carpeta,'corridas_rozamiento_limpias.mat'),...
    'corridas','resumen','todo','notas','fuente','Ts');
writetable(todo,fullfile(carpeta,'corridas_rozamiento_limpias.csv'));
writetable(resumen,fullfile(carpeta,'resumen_corridas_rozamiento.csv'));
exportgraphics(fig,fullfile(carpeta,'corridas_rozamiento_limpias.png'));
disp(resumen);
