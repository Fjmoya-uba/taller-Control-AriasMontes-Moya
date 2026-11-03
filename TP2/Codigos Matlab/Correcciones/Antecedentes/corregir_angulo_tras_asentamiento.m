function tabla = corregir_angulo_tras_asentamiento
% Hipotesis experimental independiente: cero solo tras asentarse el servo.
% No usa posicion, identificacion del carrito ni correcciones anteriores.
d=fileparts(mfilename('fullpath')); destino=fullfile(d,'Asentamiento');
if ~isfolder(destino), mkdir(destino); end
origen=fullfile(d,'..','..','Mediciones','mediciones_dinamica_carrito_1.mat');
O=load(origen); S=O;
t=double(O.out.tout(:)); ang=double(O.out.d1(:)); u=double(O.out.d3(:));
assert(all(isfinite(t)) && all(diff(t)>0) && all(isfinite(u)));
assert(numel(ang)==numel(t) && numel(u)==numel(t));
% Error normalizado de un escalon desde equilibrio:
% e(t)=exp(-alpha*t)*(cos(wd*t)+alpha/wd*sin(wd*t)).
% |e(t)| <= M*exp(-alpha*t). Exigir esa envolvente <= 2%
% garantiza permanencia en banda para todo tiempo posterior en este modelo.
den=[1 50.12 967.5]; numerador=36.82; banda=.02;
alpha=den(2)/2; wd=sqrt(den(3)-alpha^2);
M=sqrt(1+(alpha/wd)^2); ts=log(M/banda)/alpha;
centro=median(u(t<1)); activo=abs(u-centro)>1;
ini=find(diff([false;activo])==1); fin=find(diff([activo;false])==-1);
mascara=false(size(t)); transitorios=false(size(t)); filas=zeros(0,10);
for j=1:numel(ini)
    primero=fin(j)+1;
    ultimo=numel(t); if j<numel(ini), ultimo=ini(j+1)-1; end
    if primero>ultimo, continue; end
    idx=(primero:ultimo)'; umbral=t(primero)+ts;
    preservar=idx(t(idx)<umbral); cambiar=idx(t(idx)>=umbral);
    transitorios(preservar)=true; mascara(cambiar)=true;
    inicio_cero=NaN; fin_cero=NaN;
    if ~isempty(cambiar), inicio_cero=t(cambiar(1)); fin_cero=t(cambiar(end)); end
    filas(end+1,:)=[j sign(median(ang(ini(j):fin(j)))) t(primero) ...
        t(ultimo) umbral inicio_cero fin_cero numel(preservar) ...
        numel(cambiar) nnz(ang(cambiar)~=0)]; %#ok<AGROW>
end
corregido=ang; corregido(mascara)=0;
S.out.d1=reshape(cast(corregido,'like',O.out.d1),size(O.out.d1));
control=S; control.out.d1=O.out.d1;
assert(isequaln(control,O),'Se alteraron otras variables.');
assert(isequaln(S.out.d1(~mascara),O.out.d1(~mascara)));
assert(isequaln(S.out.d1(activo),O.out.d1(activo)),'Pulso alterado.');
assert(isequaln(S.out.d1(transitorios),O.out.d1(transitorios)),'Transitorio alterado.');
assert(all(S.out.d1(mascara)==0));
archivo=fullfile(destino,'mediciones_angulo_cero_tras_asentamiento.mat');
save(archivo,'-struct','S');
recargado=load(archivo); assert(isequaln(recargado,S),'Copia guardada distinta.');
assert(isequaln(load(origen),O),'Original alterado.');
tabla=array2table(filas,'VariableNames',{'episodio','signo_pulso', ...
    'retorno_centro_s','fin_descanso_s','umbral_asentamiento_s', ...
    'inicio_cero_s','fin_cero_s','muestras_transitorio_preservadas', ...
    'muestras_asignadas_cero','muestras_realmente_modificadas'});
writetable(tabla,fullfile(destino,'intervalos.csv'));
save(fullfile(destino,'auditoria.mat'),'origen','archivo','tabla','den', ...
    'numerador','banda','alpha','wd','M','ts','centro','mascara','transitorios', ...
    'activo','t','ang','corregido');
f=figure('Visible','off','Color','w','Position',[100 100 1300 750]);
subplot(3,1,1); plot(t,ang,'Color',[.65 .65 .65]); hold on;
plot(t,corregido,'b'); grid on; ylabel('Angulo [grados]');
legend('Original','Cero tras asentamiento'); title('Hipotesis experimental: pulsos y retornos preservados');
subplot(3,1,2); plot(t,u); grid on; ylabel('Comando');
subplot(3,1,3); plot(t,corregido-ang); grid on;
ylabel('Cambio [grados]'); xlabel('Tiempo [s]');
exportgraphics(f,fullfile(destino,'comparacion.png'),'Resolution',150); close(f);
f=figure('Visible','off','Color','w','Position',[100 100 1400 1000]);
for j=1:height(tabla)
    subplot(ceil(height(tabla)/2),2,j);
    plot(t,ang,'Color',[.65 .65 .65]); hold on; plot(t,corregido,'b');
    xline(tabla.retorno_centro_s(j),':k'); xline(tabla.umbral_asentamiento_s(j),'--r');
    xlim([tabla.retorno_centro_s(j)-.15 min(tabla.fin_descanso_s(j),tabla.retorno_centro_s(j)+.8)]);
    grid on; xlabel('Tiempo [s]'); ylabel('Angulo [grados]');
    title(sprintf('Retorno %d, signo %+d',j,tabla.signo_pulso(j)));
end
exportgraphics(f,fullfile(destino,'retornos.png'),'Resolution',150); close(f);
fprintf('ts conservador (2%%): %.9f s. Muestras asignadas cero: %d.\n',ts,nnz(mascara));
disp(tabla);
end
