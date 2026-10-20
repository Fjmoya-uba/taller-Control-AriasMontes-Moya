function resultados = comparar_rozamiento()
% Compara modelos con dos capturas. Solo requiere MATLAB base.
% x'' = k*sin(theta) + sesgo - c*tanh(v/0.005) - p*v
% x en m, theta en rad; c = Fc/m y p = b/m (parametros efectivos).
% Ajuste integral debil: no se calcula la segunda derivada de la posicion.
% Cada registro conserva su propio tiempo; las corridas nunca se concatenan.
carpeta=fileparts(mfilename('fullpath'));
verificar_metodo();
salida=fullfile(carpeta,'analisis_rozamiento');
if ~isfolder(salida), mkdir(salida); end
S=load(fullfile(carpeta,'mediciones_dinamica_carrito.mat'));
t=double(S.out.tout(:)); x=double(S.out.d2(:))/100;
a=double(S.out.d1(:))*pi/180;
assert(all(isfinite([t;x;a])) && all(diff(t)>0));
Ts=median(diff(t)); assert(max(abs(diff(t)-Ts))<1e-6);
[v,xsuave]=derivada_local(t,x,4);
dinamica=table(t,x,a,v,xsuave,'VariableNames',...
    {'tiempo_s','posicion_m','angulo_rad','velocidad_estimada_ms','posicion_local_m'});
tipos=["sin_rozamiento","viscoso","Coulomb","combinado"];
activos={[1 4],[1 3 4],[1 2 4],[1 2 3 4]};
% Ventanas de 0.48 s, paso 0.10 s. Validacion temporal separada por 2 s.
h=0.24;
retardos=(-0.10:0.02:0.10)';
modelos=struct([]); comparacion=table();
for m=1:numel(tipos)
    % Retardo elegido SOLO dentro del entrenamiento (3..30 s).
    errores=zeros(size(retardos));
    for d=1:numel(retardos)
        [A,y,centros]=sistema_debil(t,x,a,v,h,retardos(d));
        beta=ajustar(A(centros>=3 & centros<=21,:),...
            y(centros>=3 & centros<=21),activos{m});
        q=centros>=23 & centros<=30;
        errores(d)=sqrt(mean((A(q,:)*beta-y(q)).^2));
    end
    [~,mejor]=min(errores); retardo=retardos(mejor);
    [A,y,centros]=sistema_debil(t,x,a,v,h,retardo);
    ent=centros>=3 & centros<=30; val=centros>=32 & centros<=44;
    beta=ajustar(A(ent,:),y(ent),activos{m});
    [rmseLocal,pred]=validar_local(t,x,a,v,beta,retardo,32:1:43,0.8);
    rmseIntegral=sqrt(mean((A(val,:)*beta-y(val)).^2));
    modelos(m).nombre=tipos(m); modelos(m).beta=beta;
    modelos(m).retardo_s=retardo; modelos(m).predicciones=pred;
    modelos(m).rmse_posicion_cm=100*rmseLocal;
    modelos(m).centros_s=centros; modelos(m).aceleracion_integral=y;
    modelos(m).aceleracion_modelo=A*beta;
    fila=table(tipos(m),beta(1),beta(2),beta(3),beta(4),retardo,...
        rmseIntegral,100*rmseLocal,...
        'VariableNames',{'modelo','k_ms2','Fc_sobre_m_ms2','b_sobre_m_s1',...
        'sesgo_ms2','retardo_s','RMSE_integral_validacion_ms2','RMSE_posicion_08s_cm'});
    comparacion=[comparacion;fila]; %#ok<AGROW>
end
[~,elegido]=min(comparacion.RMSE_posicion_08s_cm);
beta=modelos(elegido).beta; retardo=modelos(elegido).retardo_s;
% Validacion externa: usar solo el interior de los tramos candidatos.
% Se quitan 0.08 s al inicio y 0.12 s al final, para evitar manos/topes.
L=load(fullfile(carpeta,'corridas_rozamiento_limpias.mat'));
F=load(fullfile(carpeta,'mediciones_dinamica_carrito_1.mat'));
tf=double(F.out.tout(:)); af=double(F.out.d1(:));
referencia_dinamica_deg=median(a(t>=46 & t<=49))*180/pi;
validacion_corridas=table(); libres=struct([]);
fig=figure('Visible','off','Position',[50 50 1500 1000]);
for r=1:numel(L.corridas)
    D=L.corridas(r).datos; D=D(D.usar_ajuste,:);
    tr=D.tiempo_original_s; xr=D.distancia_cm/100;
    inicio=L.corridas(r).inicio_inclinacion_s;
    neutral=tf>=inicio-0.8 & tf<=inicio-0.2;
    referencia_libre_deg=median(af(neutral));
    % dinamica-carrito resta el cero inicial; ensayo_rozamiento no lo hace.
    % Alinear por servo centrado, SIN ajustar el offset para mejorar el error.
    correccion_deg=referencia_dinamica_deg-referencia_libre_deg;
    ar=(D.angulo_deg+correccion_deg)*pi/180;
    q=tr>=tr(1)+0.08 & tr<=tr(end)-0.12 & isfinite(xr);
    tr=tr(q); xr=xr(q); ar=ar(q);
    pred=NaN(size(tr)); err=NaN; errCrudo=NaN; recorrido=NaN;
    if numel(tr)>=9
        [vr,~]=derivada_local(tr,xr,4);
        % Una unica simulacion por corrida, sin reinicios durante el tramo.
        pred=simular(tr,ar,beta,retardo,xr(1),vr(1));
        err=100*sqrt(mean((pred-xr).^2)); recorrido=100*abs(xr(end)-xr(1));
        predCruda=simular(tr,ar-correccion_deg*pi/180,beta,retardo,xr(1),vr(1));
        errCrudo=100*sqrt(mean((predCruda-xr).^2));
    end
    libres(r).tiempo_s=tr; libres(r).posicion_m=xr;
    libres(r).prediccion_m=pred;
    libres(r).angulo_alineado_rad=ar;
    fila=table(r,numel(tr),referencia_libre_deg,correccion_deg,errCrudo,err,recorrido,...
        'VariableNames',{'corrida','muestras_interiores','referencia_libre_deg',...
        'correccion_angulo_deg','RMSE_sin_alinear_cm','RMSE_posicion_cm','recorrido_cm'});
    validacion_corridas=[validacion_corridas;fila]; %#ok<AGROW>
    subplot(4,3,r); plot(tr,xr*100,'k.-'); hold on;
    plot(tr,pred*100,'r--'); grid on;
    title(sprintf('Corrida %d | RMSE %.2f cm',r,err));
    xlabel('Tiempo registrado [s]'); ylabel('Posicion [cm]');
end
exportgraphics(fig,fullfile(salida,'validacion_corridas.png'));
% Comparar tambien los otros modelos sobre las corridas, y ajustar con ambos
% archivos sin usar las cuatro ultimas corridas (9..12) para estimar.
comparacion_conjunta=table(); detalle_conjunto=table(); conjuntos=struct([]);
for m=1:numel(modelos)
    retraso=modelos(m).retardo_s;
    [Ad,yd,cd]=sistema_debil(t,x,a,v,h,retraso);
    q=cd>=3 & cd<=30; Ad=Ad(q,:); yd=yd(q);
    Af=[]; yf=[];
    for r=1:8
        R=libres(r);
        if numel(R.tiempo_s)<11, continue; end
        [vr,~]=derivada_local(R.tiempo_s,R.posicion_m,4);
        [Ar,yr,~]=sistema_debil(R.tiempo_s,R.posicion_m,...
            R.angulo_alineado_rad,vr,0.08,retraso);
        Af=[Af;Ar]; yf=[yf;yr]; %#ok<AGROW>
    end
    % Peso total de las corridas = mitad del peso de la captura dinamica.
    % Fijado antes de evaluar las corridas 9..12.
    factor=sqrt(0.5*size(Ad,1)/size(Af,1));
    bj=ajustar([Ad;factor*Af],[yd;factor*yf],activos{m});
    conjuntos(m).nombre=tipos(m); conjuntos(m).beta=bj;
    conjuntos(m).retardo_s=retraso;
    for ajuste=1:2
        if ajuste==1, b=modelos(m).beta; origen="solo_dinamica";
        else, b=bj; origen="ambos_archivos"; end
        [errorDinamica,~]=validar_local(t,x,a,v,b,retraso,32:1:43,0.8);
        errorLibre=[]; errorRelativo=[];
        for r=1:12
            R=libres(r); if numel(R.tiempo_s)<9, continue; end
            [vr,~]=derivada_local(R.tiempo_s,R.posicion_m,4);
            pred=simular(R.tiempo_s,R.angulo_alineado_rad,b,retraso,...
                R.posicion_m(1),vr(1));
            e=100*sqrt(mean((pred-R.posicion_m).^2));
            rango=100*(max(R.posicion_m)-min(R.posicion_m));
            fila=table(tipos(m),origen,r,e,100*e/max(rango,0.1),...
                'VariableNames',{'modelo','ajuste','corrida','RMSE_cm','RMSE_sobre_rango_pct'});
            detalle_conjunto=[detalle_conjunto;fila]; %#ok<AGROW>
            if r>=9, errorLibre(end+1)=e; errorRelativo(end+1)=100*e/max(rango,0.1); end %#ok<AGROW>
            if ajuste==2
                conjuntos(m).libres(r).prediccion_m=pred;
            end
        end
        fila=table(tipos(m),origen,b(1),b(2),b(3),b(4),100*errorDinamica,...
            median(errorLibre),median(errorRelativo),...
            'VariableNames',{'modelo','ajuste','k_ms2','Fc_sobre_m_ms2',...
            'b_sobre_m_s1','sesgo_ms2','RMSE_dinamica_cm',...
            'mediana_RMSE_corridas_9a12_cm','mediana_RMSE_relativo_9a12_pct'});
        comparacion_conjunta=[comparacion_conjunta;fila]; %#ok<AGROW>
    end
end
writetable(comparacion_conjunta,fullfile(salida,'comparacion_ambos_archivos.csv'));
writetable(detalle_conjunto,fullfile(salida,'errores_por_corrida_todos_modelos.csv'));
% Sensibilidad: ajustar de nuevo con otros anchos, sin tocar la validacion.
sensibilidad=table();
for ancho=[0.18 0.24 0.30]
    [A,y,centros]=sistema_debil(t,x,a,v,ancho,retardo);
    q=centros>=3 & centros<=30;
    b=ajustar(A(q,:),y(q),activos{elegido});
    [rmse,~]=validar_local(t,x,a,v,b,retardo,32:1:43,0.8);
    fila=table(ancho,b(1),b(2),b(3),b(4),100*rmse,...
        'VariableNames',{'semiancho_s','k_ms2','Fc_sobre_m_ms2',...
        'b_sobre_m_s1','sesgo_ms2','RMSE_posicion_08s_cm'});
    sensibilidad=[sensibilidad;fila]; %#ok<AGROW>
end
fig=figure('Visible','off','Position',[50 50 1200 850]);
subplot(2,1,1); plot(t,x*100,'k'); hold on;
colores=lines(numel(modelos));
for m=1:numel(modelos)
    P=modelos(m).predicciones;
    plot(P(:,1),P(:,2)*100,'Color',colores(m,:));
end
xlim([32 44]); grid on; ylabel('Posicion [cm]');
legend(["Medida",tipos],'Location','best');
title('Validacion temporal: predicciones de 0.8 s; reinicio cada 1 s');
subplot(2,1,2); M=modelos(elegido); q=M.centros_s>=32 & M.centros_s<=44;
plot(M.centros_s(q),M.aceleracion_integral(q),'k.-'); hold on;
plot(M.centros_s(q),M.aceleracion_modelo(q),'r.-'); grid on;
xlabel('Tiempo registrado [s]'); ylabel('Aceleracion promediada [m/s^2]');
legend('Medida por integral','Modelo','Location','best');
exportgraphics(fig,fullfile(salida,'validacion_dinamica.png'));
resultados=struct('comparacion',comparacion,'modelos',modelos,...
    'indice_menor_error',elegido,'validacion_corridas',validacion_corridas,...
    'sensibilidad',sensibilidad,'libres',libres,'Ts',Ts,...
    'referencia_dinamica_deg',referencia_dinamica_deg);
resultados.comparacion_conjunta=comparacion_conjunta;
resultados.detalle_conjunto=detalle_conjunto;
resultados.modelos_conjuntos=conjuntos;
save(fullfile(salida,'resultados_rozamiento.mat'),'resultados','dinamica');
writetable(comparacion,fullfile(salida,'comparacion_modelos.csv'));
writetable(validacion_corridas,fullfile(salida,'validacion_corridas.csv'));
writetable(sensibilidad,fullfile(salida,'sensibilidad.csv'));
disp(comparacion); disp(validacion_corridas); disp(sensibilidad);
disp(comparacion_conjunta);
fprintf('Menor error local de posicion: %s\n',tipos(elegido));
end

function verificar_metodo()
% Comprobar signos y escala integral contra una trayectoria de aceleracion
% constante, y el integrador contra la solucion analitica sin rozamiento.
t=(0:0.02:4)'; x=0.1+0.03*t+0.1*t.^2; v=0.03+0.2*t;
a=asin((0.2+0.1+0.4*v)/9.81);
[A,y,~]=sistema_debil(t,x,a,v,0.24,0);
assert(max(abs(A*[9.81;0.1;0.4;0]-y))<1e-5,...
    'Fallo en la comprobacion sintetica de la ecuacion integral.');
angulo=ones(size(t))*0.04;
pred=simular(t,angulo,[9.81;0;0;0],0,0.2,0.03);
exacta=0.2+0.03*t+0.5*9.81*sin(0.04)*t.^2;
assert(max(abs(pred-exacta))<1e-9,'Fallo en el integrador.');
end

function [v,xs]=derivada_local(t,x,n)
% Polinomio cubico local; incluye extremos con ventana unilateral.
N=numel(t); v=zeros(N,1); xs=v;
for i=1:N
    j=max(1,min(i-n,N-2*n)); indices=j:min(N,j+2*n);
    p=polyfit(t(indices)-t(i),x(indices),3);
    v(i)=p(3); xs(i)=p(4);
end
end

function [A,y,centros]=sistema_debil(t,x,a,v,h,retardo)
dt=median(diff(t)); n=round(h/dt); h=n*dt;
tau=(-n:n)'*dt; u=tau/h;
phi=(1-u.^2).^3;
dp=-6*u.*(1-u.^2).^2/h;
ddp=(-6+36*u.^2-30*u.^4)/h^2;
pesos=ones(size(tau))*dt; pesos([1 end])=dt/2;
w=pesos.*phi; w=w/sum(w);
wd=pesos.*dp/sum(pesos.*phi);
wdd=pesos.*ddp/sum(pesos.*phi);
% Eliminar error de cuadratura para posicion constante y lineal.
B=[ones(size(tau)),tau]; wdd=wdd-B*((B'*B)\(B'*wdd));
wdd=wdd*(2/sum(wdd.*tau.^2));
wd=wd-mean(wd); wd=wd*(-1/sum(wd.*tau));
entrada=sin(interp1(t,a,t-retardo,'linear','extrap'));
indices=(n+1:5:numel(t)-n)'; A=zeros(numel(indices),4); y=zeros(numel(indices),1);
for k=1:numel(indices)
    j=indices(k)+(-n:n);
    A(k,:)=[sum(w.*entrada(j)), -sum(w.*tanh(v(j)/0.005)),...
        sum(wd.*x(j)),1];
    y(k)=sum(wdd.*x(j));
end
centros=t(indices);
end

function b=ajustar(A,y,activos)
% Minimos cuadrados robustos con k,c,p>=0 y sesgo libre.
pesos=ones(size(y)); b=zeros(4,1);
for iter=1:6
    mejor=Inf; restringidos=intersect(activos,1:3);
    for bits=0:2^numel(restringidos)-1
        libres=activos;
        for j=1:numel(restringidos)
            if bitget(bits,j), libres(libres==restringidos(j))=[]; end
        end
        C=A(:,libres).*sqrt(pesos); z=y.*sqrt(pesos);
        escala=sqrt(sum(C.^2,1)); escala(escala==0)=1;
        candidato=zeros(4,1); candidato(libres)=(C./escala)\z;
        candidato(libres)=candidato(libres)./escala';
        if any(candidato(restringidos)<-1e-10), continue; end
        error=sum(pesos.*(A*candidato-y).^2);
        if error<mejor, mejor=error; b=candidato; end
    end
    residuo=A*b-y; s=1.4826*median(abs(residuo-median(residuo)))+1e-9;
    pesos=min(1,1.5*s./max(abs(residuo),1e-12));
end
end

function [rmse,P]=validar_local(t,x,a,v,b,retardo,inicios,duracion)
P=zeros(0,2); residuos=[];
for inicio=inicios
    indices=find(t>=inicio & t<=inicio+duracion);
    % Condiciones iniciales estimadas solo con las 9 muestras precedentes.
    i=indices(1); j=max(1,i-8):i;
    p=polyfit(t(j)-t(i),x(j),2); x0=p(3); v0=p(2);
    pred=simular(t(indices),a(indices),b,retardo,x0,v0,...
        t,a);
    residuos=[residuos;pred-x(indices)]; %#ok<AGROW>
    P=[P;t(indices),pred;NaN NaN]; %#ok<AGROW>
end
rmse=sqrt(mean(residuos.^2));
end

function pred=simular(t,a,b,retardo,x0,v0,tEntrada,aEntrada)
if nargin<8, tEntrada=t; aEntrada=a; end
f=griddedInterpolant(tEntrada,aEntrada,'linear','nearest');
pred=zeros(size(t)); pred(1)=x0; estado=[x0;v0];
for k=2:numel(t)
    n=ceil((t(k)-t(k-1))/0.002); dt=(t(k)-t(k-1))/n;
    for s=1:n
        tiempo=t(k-1)+(s-1)*dt;
        k1=rhs(tiempo,estado); k2=rhs(tiempo+dt/2,estado+dt*k1/2);
        k3=rhs(tiempo+dt/2,estado+dt*k2/2); k4=rhs(tiempo+dt,estado+dt*k3);
        estado=estado+dt*(k1+2*k2+2*k3+k4)/6;
    end
    pred(k)=estado(1);
end
    function dz=rhs(tiempo,z)
        dz=[z(2);b(1)*sin(f(tiempo-retardo))+b(4)-...
            b(2)*tanh(z(2)/0.005)-b(3)*z(2)];
    end
end
