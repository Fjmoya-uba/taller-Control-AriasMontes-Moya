function evaluar_modelos
% Comparacion reproducible. Agregar candidatos a esta tabla y ejecutar.
d=fileparts(mfilename('fullpath')); med=fullfile(d,'..','..','Mediciones');
salida=fullfile(d,'Metricas'); if ~isfolder(salida), mkdir(salida); end
modelos=table(["Original_TP";"Residual_corregido"], ...
    [6.815007;3.2683171],[24.428153;13.487667],[-.56922;-1.7860731], ...
    'VariableNames',{'modelo','a','K','c'});
modelos.tau_s=1./modelos.a; modelos.K_sobre_a=modelos.K./modelos.a;
writetable(modelos,fullfile(salida,'parametros.csv'));
% Coeficientes del informe, sin resintonizar por planta.
controladores=["P";"PI";"PD"];
bs={[19.95262315 0 0],[12.6522004 -12.5263078 0],[15.4003772 -14.5032679 0]};
as={[1 0 0],[1 -1 0],[1 -.886792453 0]};
archivos=["controlador_proporcional";"controlador_proporcional_integral"; ...
    "controlador_proporcional_derivativo"];
config_control=table(controladores,bs',as','VariableNames',{'controlador','b','a'});
save(fullfile(salida,'configuracion.mat'),'modelos','config_control');
filas={}; pendientes={}; curvas=struct; golpes={};
% Lazo abierto: misma posicion, ventana y condiciones que la identificacion.
S=load(fullfile(med,'mediciones_dinamica_carrito_1.mat')); o=S.out;
SC=load(fullfile(d,'..','mediciones_dinamica_carrito_1_corregida.mat'));
t=double(o.tout(:)); y=double(o.d2(:))-15.612; u=double(o.d3(:));
ok=t<=31.8 & isfinite(y)&y>=-15&y<=21;
fin=find(~ok,1)-1; if isempty(fin), fin=numel(t); end
t=t(1:fin); y=y(1:fin); u=u(1:fin); ne=floor(.75*fin);
entradas={double(o.d1(1:fin)),double(SC.out.d1(1:fin))};
assert(numel(entradas)==height(modelos),'Definir una entrada angular por candidato.');
centro=median(u(t<1)); act=abs(u-centro)>1;
ini=find(diff([false;act])==1); ends=find(diff([act;false])==-1);
yp=zeros(fin,height(modelos));
for m=1:height(modelos)
    p=modelos(m,:); ang=entradas{m}(:);
    sys=ss([0 1;0 -p.a],[0;1],[1 0],0);
    yp(:,m)=.2995+lsim(sys,p.K*ang+p.c,t,[0;0],'zoh');
    filas(end+1,:)=fila(p.modelo,"Abierto","identificacion","c_identificado", ...
        t(ne+1:end),y(ne+1:end),yp(ne+1:end,m),0,NaN); %#ok<AGROW>
    for j=1:numel(ini)
        primero=ends(j)+1;
        if j<numel(ini), ultimo=ini(j+1)-1; else, ultimo=fin; end
        if primero>ultimo, continue; end
        idx=(primero:ultimo)'; idx=idx(t(idx)>=t(primero)+.8);
        if numel(idx)<10, continue; end
        pr=polyfit(t(idx)-t(idx(1)),y(idx),1);
        ps=polyfit(t(idx)-t(idx(1)),yp(idx,m),1);
        signo=sign(median(double(o.d1(ini(j):ends(j)))));
        pendientes(end+1,:)={p.modelo,j,signo,t(idx(1)),t(idx(end)), ...
            t(idx(1))>t(ne),pr(1),ps(1),ps(1)-pr(1),abs(ps(1)-pr(1))}; %#ok<AGROW>
    end
end
grafico(salida,'abierto',t,y,yp,modelos.modelo,zeros(size(t)));
curvas.abierto=struct('t',t,'y',y,'predicciones',yp,'entradas',{entradas});
for tipo=["escalon","golpe"]
 for cidx=1:3
    archivo=fullfile(med,archivos(cidx)+"_"+tipo+".mat");
    S=load(archivo); o=S.out;
    t=double(o.tout(:)); y=double(o.d2(:))-15.612;
    r=double(o.d1(:))-15.612; y(~isfinite(y)|double(o.d2(:))<=0)=NaN;
    salto=0;
    if tipo=="escalon"
        inicio=find(abs(diff(r))>5,1)+1;
        assert(~isempty(inicio),'Escalon no detectado.');
        objetivo=r(inicio); instante=t(inicio);
    else
        base=t<=t(1)+1 & isfinite(y); ybase=median(y(base));
        ruido=1.4826*median(abs(y(base)-ybase)); umbral=max(.4,6*ruido);
        fuera=t>t(1)+1 & isfinite(y)&abs(y-ybase)>umbral;
        inicio=find(conv(double(fuera),ones(3,1),'valid')==3,1);
        assert(~isempty(inicio),'Golpe no detectado.'); instante=t(inicio); objetivo=0;
        sentido=sign(median(y(t>=instante & t<=instante+.3),'omitnan')-ybase);
        if sentido==0, sentido=1; end
        salto=5*sentido; % Igual para todas las plantas; amplitud real desconocida.
        golpes(end+1,:)={controladores(cidx),instante,salto,umbral}; %#ok<AGROW>
    end
    t=t-instante; post=t>=0; t=t(post); y=y(post); r=r(post);
    preds=[]; nombres=strings(0,1);
    for sesgo=[false true]
     for m=1:height(modelos)
        p=modelos(m,:); cvalor=double(sesgo)*p.c;
        [ts,ys,us]=simular(p,bs{cidx},as{cidx},t(end),objetivo,salto,cvalor);
        pred=interp1(ts,ys,t,'linear');
        assert(all(isfinite(pred)),'La simulacion no cubre todas las muestras.');
        escenario="nominal_c0"; if sesgo, escenario="c_identificado"; end
        filas(end+1,:)=fila(p.modelo,controladores(cidx),tipo,escenario, ...
            t,y,pred,objetivo,sum(us<=1100|us>=2000)); %#ok<AGROW>
        preds(:,end+1)=pred; nombres(end+1)=p.modelo+" / "+escenario; %#ok<AGROW>
     end
    end
    clave=char(controladores(cidx)+"_"+tipo);
    grafico(salida,clave,t,y,preds,nombres,r);
    curvas.(clave)=struct('t',t,'y',y,'predicciones',preds,'nombres',nombres, ...
        'instante_alineacion',instante,'salto_velocidad',salto);
 end
end
T=cell2table(filas,'VariableNames',{'modelo','controlador','ensayo','escenario', ...
    'RMSE_cm','FIT_pct','error_medio_sim_menos_real_cm','t_subida_real_s', ...
    't_subida_sim_s','delta_t_subida_s','sobrepaso_real_pct','sobrepaso_sim_pct', ...
    'delta_sobrepaso_pp','final_real_cm','final_sim_cm','delta_final_cm', ...
    'establecimiento_real_s','establecimiento_sim_s','delta_establecimiento_s', ...
    'max_abs_real_cm','max_abs_sim_cm','delta_max_abs_cm','muestras_limite_sim'});
writetable(T,fullfile(salida,'metricas_detalladas.csv'));
P=cell2table(pendientes,'VariableNames',{'modelo','episodio','signo','inicio_s','fin_s', ...
    'solo_validacion','pendiente_real_cm_s','pendiente_sim_cm_s','error_cm_s','error_abs_cm_s'});
writetable(P,fullfile(salida,'pendientes_descansos.csv'));
G=cell2table(golpes,'VariableNames',{'controlador','instante_detectado_s','salto_supuesto_cm_s','umbral_cm'});
writetable(G,fullfile(salida,'configuracion_golpes.csv'));
save(fullfile(salida,'evaluacion.mat'),'T','P','G','curvas','modelos');
disp(T(:,1:6));
% Tabla compacta de RMSE: abierto con c identificado, controles nominales.
R=T(T.controlador=="Abierto"|T.escenario=="nominal_c0",1:5);
R.prueba=R.controlador+"_"+R.ensayo; R=R(:,{'modelo','prueba','RMSE_cm'});
R=unstack(R,'RMSE_cm','prueba');
R.MAE_pendiente_positiva_cm_s=NaN(height(R),1);
R.MAE_pendiente_negativa_cm_s=NaN(height(R),1);
for k=1:height(R)
    mismo=P.modelo==R.modelo(k);
    R.MAE_pendiente_positiva_cm_s(k)=mean(P.error_abs_cm_s(mismo & P.signo>0));
    R.MAE_pendiente_negativa_cm_s(k)=mean(P.error_abs_cm_s(mismo & P.signo<0));
end
writetable(R,fullfile(salida,'resumen_RMSE.csv'));
end

function [t,y,u]=simular(p,b,a,fin,ref,v0,c)
% Una muestra extra evita NaN al interpolar el extremo por redondeo temporal.
Ts=.02; t=(0:Ts:(ceil(fin/Ts)+1)*Ts)';
sys=ss([0 1 0 0;0 -p.a p.K 0;0 0 0 1;0 0 -967.5 -50.12], ...
    [0 0;0 1;0 0;36.82 0],[1 0 0 0],[0 0]);
sys=c2d(sys,Ts,'zoh'); x=[0;v0;0;0]; me=zeros(2,1); mu=me;
y=zeros(size(t)); u=y;
for k=1:numel(t)
    y(k)=x(1); e=ref-y(k); s=-a(2:3)*mu+b*[e;me];
    limitado=min(max(1544+s,1100),2000); u(k)=round(limitado);
    me=[e;me(1)]; mu=[limitado-1544;mu(1)];
    x=sys.A*x+sys.B*[u(k)-1544;c];
end
end

function f=fila(modelo,control,ensayo,esc,t,y,yp,r,sat)
ok=isfinite(y)&isfinite(yp); e=yp(ok)-y(ok);
den=norm(y(ok)-mean(y(ok))); fit=NaN;
if den>eps, fit=100*(1-norm(e)/den); end
mr=medidas(t,y,r,ensayo); ms=medidas(t,yp,r,ensayo);
f={modelo,control,ensayo,esc,sqrt(mean(e.^2)),fit,mean(e), ...
    mr(1),ms(1),ms(1)-mr(1),mr(2),ms(2),ms(2)-mr(2), ...
    mr(3),ms(3),ms(3)-mr(3),mr(4),ms(4),ms(4)-mr(4), ...
    mr(5),ms(5),ms(5)-mr(5),sat};
end

function m=medidas(t,y,r,tipo)
ok=isfinite(y); cola=t>=t(end)-2 & ok; final=mean(y(cola));
subida=NaN; sobre=NaN; est=NaN; pico=max(abs(y(ok)));
if tipo=="escalon"
    i10=find(ok & y>=.1*r,1); i90=find(ok & y>=.9*r,1);
    if ~isempty(i10)&&~isempty(i90), subida=t(i90)-t(i10); end
    sobre=max(0,(max(y(ok))-r)/abs(r)*100); banda=.02*abs(r);
elseif tipo=="golpe", banda=.2;
else, m=[subida sobre final est pico]; return;
end
fuera=~ok|abs(y-r)>banda; ultimo=find(fuera,1,'last');
if isempty(ultimo), est=t(1); elseif ultimo<numel(t), est=t(ultimo+1); end
m=[subida sobre final est pico];
end

function grafico(d,nombre,t,y,preds,nombres,r)
f=figure('Visible','off','Color','w','Position',[100 100 1200 650]);
plot(t,y,'k','LineWidth',1.3); hold on; plot(t,preds); plot(t,r,':k'); grid on;
xlabel('Tiempo [s]'); ylabel('Posicion [cm]'); title(strrep(nombre,'_',' '));
legend(['Real';nombres(:);'Referencia'],'Interpreter','none','Location','best');
exportgraphics(f,fullfile(d,[char(nombre) '.png']),'Resolution',140); close(f);
end
