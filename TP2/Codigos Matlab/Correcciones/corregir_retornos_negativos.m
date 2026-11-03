function corregir_retornos_negativos
% Copia experimental: solo d1 en descansos posteriores a pulsos negativos.
% No estima modelos. Parametros editables, independientes del polo buscado.
duracion_transicion = 0.6; % s, subida suave de la correccion
% Al proximo pulso se retoma la medicion original sin alterar el pulso.
ventana_residual = 0.6;   % s, mediana al final del descanso
max_descanso_referencia = 3; % s, evita usar la cola larga tras finalizar
carpeta = fileparts(mfilename('fullpath'));
origen = fullfile(carpeta,'..','Mediciones','mediciones_dinamica_carrito_1.mat');
S = load(origen); original = S;
t = double(S.out.tout(:)); a = double(S.out.d1(:));
u = double(S.out.d3(:)); y = double(S.out.d2(:))-15.612;
assert(all(isfinite(t)) && all(diff(t)>0),'Tiempo invalido.');
centro = median(u(t<1)); activo = abs(u-centro)>1;
inicios = find(diff([false;activo])==1);
finales = find(diff([activo;false])==-1);
correccion = zeros(size(a)); registro = []; descansos = {};
for k=1:numel(inicios)
    if median(a(inicios(k):finales(k)))>=0, continue; end
    primero=finales(k)+1;
    if k<numel(inicios), ultimo=inicios(k+1)-1; else, ultimo=numel(t); end
    if ultimo<=primero, continue; end
    fin_ref=min(t(ultimo),t(primero)+max_descanso_referencia);
    ref=t>=max(t(primero)+duracion_transicion,fin_ref-ventana_residual) & t<=fin_ref;
    assert(nnz(ref)>=5,'Descanso demasiado corto para estimar residual.');
    residual=median(a(ref),'omitnan');
    if ~isfinite(residual) || residual>=0, continue; end
    idx=(primero:ultimo)'; tau=t(idx)-t(primero);
    r=min(1,max(0,tau/duracion_transicion)); peso=r.^2.*(3-2*r);
    correccion(idx)=-residual*peso;
    registro(end+1,:)=[t(primero),t(ultimo),residual,median(a(ref)+correccion(ref))]; %#ok<AGROW>
    descansos{end+1}=idx; %#ok<AGROW>
end
corregido=a+correccion;
S.out.d1=reshape(cast(corregido,'like',S.out.d1),size(S.out.d1));
% Preservar todas las variables y campos originales; auditoria por separado.
destino=fullfile(carpeta,'mediciones_dinamica_carrito_1_corregida.mat');
save(destino,'-struct','S');
control=S; control.out.d1=original.out.d1;
assert(isequaln(control,original),'Se alteraron otras variables.');
assert(isequaln(corregido(activo),a(activo)),'Se altero un pulso activo.');
tabla=array2table(registro,'VariableNames', ...
    {'inicio_descanso_s','fin_descanso_s','residual_original_grados','residual_corregido_grados'});
writetable(tabla,fullfile(carpeta,'correcciones_aplicadas.csv')); disp(tabla);
save(fullfile(carpeta,'auditoria_correccion.mat'),'origen','destino','correccion', ...
    'tabla','duracion_transicion','ventana_residual','max_descanso_referencia');
fig=figure('Visible','off','Color','w','Position',[100 100 1200 850]);
subplot(3,1,1); plot(t,a,'Color',[.6 .6 .6]); hold on;
plot(t,corregido,'b'); yline(0,':k'); grid on; xlim([0 31.8]);
ylabel('Angulo [grados]'); legend('Original','Copia corregida');
title('Correccion del residual en descansos negativos; pulsos intactos');
subplot(3,1,2); plot(t,correccion); grid on; xlim([0 31.8]);
ylabel('Correccion sumada [grados]');
subplot(3,1,3); plot(t,y,'k'); grid on; xlim([0 31.8]);
ylabel('Posicion medida [cm]'); xlabel('Tiempo [s]');
exportgraphics(fig,fullfile(carpeta,'angulo_corregido.png'),'Resolution',160); close(fig);
fig=figure('Visible','off','Color','w','Position',[100 100 1200 800]);
for k=1:numel(descansos)
    idx=descansos{k}; limites=[max(0,t(idx(1))-.25),min(t(idx(end))+.15,31.8)];
    subplot(ceil(numel(descansos)/2),2,k);
    plot(t,a,'Color',[.6 .6 .6]); hold on; plot(t,corregido,'b');
    yline(0,':k'); xline(t(idx(1)),':'); grid on; xlim(limites);
    ylim([-1.5 .5]); xlabel('Tiempo [s]'); ylabel('Angulo [grados]');
    title(sprintf('Retorno negativo %d',k));
end
exportgraphics(fig,fullfile(carpeta,'detalle_retornos_corregidos.png'),'Resolution',160); close(fig);
fprintf('Copia guardada: %s\nOriginal y pulsos activos verificados intactos.\n',destino);
end

