function resumen = comparar_identificaciones
% Experimentos aislados. Ejecutar este archivo desde cualquier directorio.
% Requiere MATLAB y Control System Toolbox. No modifica datos originales.
carpeta = fileparts(mfilename('fullpath'));
archivo = fullfile(carpeta,'..','..','Mediciones','mediciones_dinamica_carrito_1.mat');
S = load(archivo); o = S.out;
t = double(o.tout(:)); angulo = double(o.d1(:));
y = double(o.d2(:))-15.612; u = double(o.d3(:));
valida = isfinite(t)&isfinite(angulo)&isfinite(y)&isfinite(u)& ...
    y>=-15 & y<=21 & t<=31.8;
fin = find(~valida,1)-1;
if isempty(fin), fin=numel(t); end
assert(fin>=100 && abs(t(1))<1e-9,'Revisar inicio y continuidad del registro.');
t=t(1:fin); angulo=angulo(1:fin); y=y(1:fin); u=u(1:fin);
assert(max(abs(diff(t)-median(diff(t))))<1e-6,'Muestreo irregular.');
n=numel(t); ne=floor(.75*n); entrenamiento=(1:ne)'; validacion=(ne+1:n)';
y0=15.9115-15.612;
u0=median(u(t<1));
Gservo=tf(36.82,[1 50.12 967.5]); % Mismo modelo que disenar_control.m
% d3 es el comando retenido vigente en cada muestra, segun el Arduino.
angulo_modelado=lsim(Gservo,u-u0,t,[],'zoh');

% Episodios: pulso completo y descanso posterior, sin pegar muestras.
activo=abs(u-u0)>1;
inicios=find(diff([false;activo])==1);
finales=find(diff([activo;false])==-1);
segmentos={}; filas_episodios=[];
for j=1:numel(inicios)
    i=inicios(j); f=finales(j);
    if j<numel(inicios), ultimo=inicios(j+1)-1; else, ultimo=n; end
    signo=sign(median(angulo(i:f)));
    completo=ultimo<=ne && f<ultimo;
    filas_episodios(end+1,:)=[t(i),t(f),t(ultimo),signo,completo]; %#ok<AGROW>
    if signo>0 && completo && ultimo-i>=20
        segmentos{end+1}=(i:ultimo)'; %#ok<AGROW>
    end
end
assert(~isempty(segmentos),'No hay episodios positivos completos de entrenamiento.');
writetable(array2table(filas_episodios,'VariableNames', ...
    {'inicio_s','fin_pulso_s','fin_episodio_s','signo_angulo','completo_entrenamiento'}), ...
    fullfile(carpeta,'episodios.csv'));

entradas={angulo,angulo,angulo_modelado};
nombres=["Base_angulo_medido";"Episodios_positivos";"Servo_fijo_comando"];
grilla=unique([0 logspace(-3,2,100) 3 3.5 4]);
perfiles=zeros(numel(grilla),3); predicciones=zeros(n,3);
modelos=cell(3,1); A=zeros(3,1); K=A; C=A; fit_est=A; fit_val=A; rmse_val=A;
for caso=1:3
    entrada=entradas{caso};
    objetivo=@(a) ajustar(a,entrada,t,y,y0,entrenamiento,segmentos,caso==2);
    for j=1:numel(grilla), perfiles(j,caso)=objetivo(grilla(j)); end
    [~,j]=min(perfiles(:,caso)); a=grilla(j);
    if j>1 && j<numel(grilla)
        candidato=fminbnd(objetivo,grilla(j-1),grilla(j+1));
        if objetivo(candidato)<objetivo(a), a=candidato; end
    end
    [~,coef]=objetivo(a); A(caso)=a; K(caso)=coef(1); C(caso)=coef(2);
    % Validacion comun: simulacion continua, y0 original, v0=0.
    predicciones(:,caso)=y0+bases(a,t,entrada)*coef(1:2);
    fit_est(caso)=fit(y(entrenamiento),predicciones(entrenamiento,caso));
    fit_val(caso)=fit(y(validacion),predicciones(validacion,caso));
    rmse_val(caso)=sqrt(mean((y(validacion)-predicciones(validacion,caso)).^2));
    modelos{caso}=tf(K(caso),[1 a 0]);
end
resumen=table(nombres,A,K,C,fit_est,fit_val,rmse_val,'VariableNames', ...
    {'experimento','a_polo','K','c','FIT_entrenamiento_comun','FIT_validacion','RMSE_validacion_cm'});
disp(resumen); writetable(resumen,fullfile(carpeta,'resumen.csv'));
save(fullfile(carpeta,'resultados.mat'),'resumen','modelos','Gservo','t','y', ...
    'angulo','angulo_modelado','u','u0','predicciones','grilla','perfiles','segmentos','archivo');
fig=figure('Visible','off','Color','w');
subplot(3,1,1); plot(t,angulo,t,angulo_modelado,'--'); grid on;
ylabel('Angulo [grados]'); legend('Medido','Servo fijo');
subplot(3,1,2); plot(t,y,'k',t,predicciones); grid on;
xline(t(ne+1),':k'); ylabel('Posicion [cm]'); legend(['Medida';nombres],'Interpreter','none');
subplot(3,1,3); semilogx(grilla(2:end),perfiles(2:end,:)); grid on;
xline(3,':'); xline(4,':'); xlabel('a [1/s], polo = -a'); ylabel('ECM de ajuste');
exportgraphics(fig,fullfile(carpeta,'comparacion.png'),'Resolution',160); close(fig);
end

function [ecm,coef]=ajustar(a,u,t,y,y0,entrenamiento,segmentos,por_episodios)
if ~por_episodios
    B=bases(a,t,u); B=B(entrenamiento,:); objetivo=y(entrenamiento)-y0;
else
    B=[]; objetivo=[];
    for k=1:numel(segmentos)
        idx=segmentos{k}; tau=t(idx)-t(idx(1));
        bloque=bases(a,tau,u(idx));
        if a<1e-10, rv=tau; else, rv=-expm1(-a*tau)/a; end
        velocidades=zeros(numel(idx),numel(segmentos)); velocidades(:,k)=rv;
        B=[B;bloque velocidades]; %#ok<AGROW>
        objetivo=[objetivo;y(idx)-y(idx(1))]; %#ok<AGROW>
    end
end
coef=B\objetivo; ecm=mean((B*coef-objetivo).^2);
end

function B=bases(a,t,u)
sys=ss([0 1;0 -a],[0;1],[1 0],0);
B=[lsim(sys,u,t,[0;0],'zoh'),lsim(sys,ones(size(u)),t,[0;0],'zoh')];
end

function valor=fit(y,yp)
den=norm(y-mean(y));
if den<eps, valor=NaN; else, valor=100*(1-norm(y-yp)/den); end
end
