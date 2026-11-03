function R = evaluar_variantes_asentamiento
% Tanda aislada: 4 entradas x 4 ajustes x 2 intervalos de a.
d=fileparts(mfilename('fullpath')); destino=fullfile(d,'Asentamiento','Variantes');
if ~isfolder(destino), mkdir(destino); end
O=load(fullfile(d,'..','..','Mediciones','mediciones_dinamica_carrito_1.mat'));
C=load(fullfile(d,'..','mediciones_dinamica_carrito_1_corregida.mat'));
Z=load(fullfile(d,'Variantes','mediciones_descansos_cero.mat'));
S=load(fullfile(d,'Asentamiento','mediciones_angulo_cero_tras_asentamiento.mat'));
A=load(fullfile(d,'Asentamiento','auditoria.mat'),'ts');
t=double(O.out.tout(:)); y=double(O.out.d2(:))-15.612; u=double(O.out.d3(:));
entradas=[double(O.out.d1(:)) double(C.out.d1(:)) double(Z.out.d1(:)) double(S.out.d1(:))];
ok=isfinite(t)&isfinite(y)&isfinite(u)&all(isfinite(entradas),2)&y>=-15&y<=21&t<=31.8;
f=find(~ok,1); if isempty(f), f=numel(t)+1; end
n=f-1; t=t(1:n); y=y(1:n); u=u(1:n); entradas=entradas(1:n,:);
dt=median(diff(t)); assert(abs(t(1))<1e-9 && max(abs(diff(t)-dt))<1e-6);
ne=floor(.75*n); train=(1:ne)'; val=(ne+1:n)'; y0=.2995;
% Contraste independiente de la recurrencia ZOH contra Control Toolbox.
for aa=[0 1e-6 3.5 100]
    [pp,vv]=respuesta(aa,dt,entradas(:,4));
    ref=lsim(ss([0 1;0 -aa],[0;1],eye(2),zeros(2,1)),entradas(:,4),t,[0;0],'zoh');
    assert(max(abs([pp vv]-ref),[],'all')<1e-7,'Recurrencia ZOH inconsistente.');
end
activo=abs(u-median(u(t<1)))>1; ini=find(diff([false;activo])==1);
fin=find(diff([activo;false])==-1); grupo=zeros(n,1); episodio=grupo; asentado=false(n,1);
for j=1:numel(ini)
    ultimo=n; if j<numel(ini), ultimo=ini(j+1)-1; end
    signo=sign(median(entradas(ini(j):fin(j),1))); assert(signo~=0);
    grupo(ini(j):fin(j))=1+(signo<0); grupo(fin(j)+1:ultimo)=3+(signo<0);
    episodio(ini(j):ultimo)=j;
    if fin(j)<ultimo
        idx=(fin(j)+1:ultimo)'; asentado(idx)=t(idx)>=t(fin(j)+1)+A.ts;
    end
end
% Derivada local cuadratica centrada, ventanas fijadas antes de comparar.
% Se calcula por particion, sin mezclar entrenamiento y validacion.
ventanas=[11 21]; vref=NaN(n,2);
for h=1:2
    m=(ventanas(h)-1)/2; x=(-m:m)'*dt; Q=[ones(size(x)) x x.^2]; D=pinv(Q);
    for parte=1:2
        idx=train; if parte==2, idx=val; end
        for j=m+1:numel(idx)-m
            vref(idx(j),h)=D(2,:)*y(idx(j-m:j+m));
        end
    end
end
fuentes=["Original","Constantes","Cero_inmediato","Asentamiento"];
tipos=["base","c0","v0_libre","equilibrado"]; rangos=["libre","a_2_5"];
filas={}; pos={}; vel={}; pendientes={}; perfiles=cell(32,1); grillas=perfiles;
pred=zeros(n,32); velocidad=pred; pesos_guardados=zeros(ne,32); k=0;
for fuente=1:4
 for tipo=1:4
  for rango=1:2
    k=k+1; nombre=fuentes(fuente)+"_"+tipos(tipo)+"_"+rangos(rango);
    w=ones(ne,1);
    if tipo==4
        for g=0:4
            ii=grupo(train)==g; assert(any(ii)); w(ii)=1/nnz(ii);
        end
        w=w/mean(w);
    end
    limites=[0 100]; grilla=unique([0 logspace(-3,2,150) linspace(2,5,61)]);
    if rango==2, limites=[2 5]; grilla=linspace(2,5,121); end
    obj=@(a) ajustar(a,dt,t,entradas(:,fuente),y,y0,train,w,tipo);
    [a,perfil]=buscar(obj,grilla); [ecm,coef,B,V]=obj(a);
    pred(:,k)=y0+B*coef; velocidad(:,k)=V*coef;
    K=coef(1); c=0; v0=0;
    if tipo~=2, c=coef(2); end
    if tipo==3, v0=coef(3); end
    [rt,bt]=metrica(y(train),pred(train,k)); [rv,bv]=metrica(y(val),pred(val,k));
    vinf=NaN; if a>1e-8, vinf=c/a; end
    filas(end+1,:)={nombre,fuentes(fuente),tipos(tipo),rangos(rango),a,-a,K,c,v0, ...
        vinf,rt,bt,rv,bv,ecm,min(abs(a-limites))<1e-5,K<=0}; %#ok<AGROW>
    grillas{k}=grilla; perfiles{k}=perfil; pesos_guardados(:,k)=w;
    for parte=0:2
        idx=(1:n)'; if parte==1, idx=train; elseif parte==2, idx=val; end
        for g=-1:4
            ii=idx; if g>=0, ii=idx(grupo(idx)==g); end
            if isempty(ii), continue; end
            [rm,b]=metrica(y(ii),pred(ii,k));
            pos(end+1,:)={nombre,parte,g,numel(ii),rm,b}; %#ok<AGROW>
            for h=1:2
                jj=ii(isfinite(vref(ii,h))); if isempty(jj), continue; end
                [rm,b]=metrica(vref(jj,h),velocidad(jj,k));
                vel(end+1,:)={nombre,parte,g,ventanas(h),numel(jj),rm,b}; %#ok<AGROW>
            end
        end
    end
    for j=1:numel(ini)
     for parte=1:2
        idx=train; if parte==2, idx=val; end
        ii=idx(episodio(idx)==j & asentado(idx)); if numel(ii)<10, continue; end
        pr=polyfit(t(ii)-t(ii(1)),y(ii),1); ps=polyfit(t(ii)-t(ii(1)),pred(ii,k),1);
        pendientes(end+1,:)={nombre,j,parte,grupo(ii(1)),t(ii(1)),t(ii(end)), ...
            pr(1),ps(1),ps(1)-pr(1),mean(velocidad(ii,k))}; %#ok<AGROW>
     end
    end
  end
 end
end
R=cell2table(filas,'VariableNames',{'modelo','entrada','ajuste','busqueda','a','polo','K','c','v0', ...
    'v_equilibrio_angulo_cero','RMSE_train','sesgo_train','RMSE_val','sesgo_val','objetivo_train','en_limite','K_no_fisica'});
P=cell2table(pos,'VariableNames',{'modelo','particion','grupo','N','RMSE_cm','sesgo_cm'});
V=cell2table(vel,'VariableNames',{'modelo','particion','grupo','ventana_muestras','N','RMSE_cm_s','sesgo_cm_s'});
L=cell2table(pendientes,'VariableNames',{'modelo','episodio','particion','grupo','inicio_s','fin_s', ...
    'pendiente_real','pendiente_sim','error_cm_s','velocidad_media_sim'});
writetable(R,fullfile(destino,'resumen.csv')); writetable(P,fullfile(destino,'posicion.csv'));
writetable(V,fullfile(destino,'velocidad.csv')); writetable(L,fullfile(destino,'pendientes.csv'));
% Estabilidad: excluir un episodio completo de entrenamiento por vez.
% Se preserva la simulacion continua, sin ajustar estados por episodio.
est={}; completos=[];
for j=1:numel(ini)
    ii=find(episodio==j); if ~isempty(ii) && max(ii)<=ne, completos(end+1)=j; end %#ok<AGROW>
end
for fuente=[1 4]
 for tipo=[1 3]
  for rango=1:2
   for j=completos
    idx=train(episodio(train)~=j); w=ones(size(idx));
    grilla=unique([0 logspace(-3,2,100) linspace(2,5,31)]);
    if rango==2, grilla=linspace(2,5,81); end
    obj=@(a) ajustar(a,dt,t,entradas(:,fuente),y,y0,idx,w,tipo);
    [a,~]=buscar(obj,grilla); [~,coef]=obj(a); v0=0; if tipo==3, v0=coef(3); end
    est(end+1,:)={fuentes(fuente),tipos(tipo),rangos(rango),j,a,coef(1),coef(2),v0}; %#ok<AGROW>
   end
  end
 end
end
E=cell2table(est,'VariableNames',{'entrada','ajuste','busqueda','episodio_excluido','a','K','c','v0'});
writetable(E,fullfile(destino,'estabilidad_entrenamiento.csv'));
save(fullfile(destino,'evaluacion.mat'),'R','P','V','L','E','t','y','entradas','pred','velocidad', ...
    'vref','ventanas','ne','y0','grupo','episodio','asentado','perfiles','grillas','pesos_guardados');
graficar_variantes_asentamiento;
disp(R); fprintf('Particion: n=%d, train hasta %.2f, val desde %.2f s.\n',n,t(ne),t(ne+1));
diagnosticar_variantes_asentamiento;
end
function [a,perfil]=buscar(obj,grilla)
perfil=arrayfun(obj,grilla); [~,j]=min(perfil); a=grilla(j);
ii=find(perfil(2:end-1)<=perfil(1:end-2)&perfil(2:end-1)<=perfil(3:end))+1;
for j=ii
    aa=fminbnd(obj,grilla(j-1),grilla(j+1)); if obj(aa)<obj(a), a=aa; end
end
end
function [ecm,coef,B,V]=ajustar(a,dt,t,u,y,y0,idx,w,tipo)
[p,v]=respuesta(a,dt,u); B=p; V=v;
if tipo~=2
    [p,v]=respuesta(a,dt,ones(size(u))); B=[B p]; V=[V v];
end
if tipo==3
    rv=t; if a>1e-10, rv=-expm1(-a*t)/a; end
    B=[B rv]; V=[V exp(-a*t)];
end
X=B(idx,:).*sqrt(w); z=(y(idx)-y0).*sqrt(w); coef=X\z;
ecm=mean((X*coef-z).^2);
end
function [p,v]=respuesta(a,dt,u)
r=exp(-a*dt);
if a*dt<1e-5
    h=dt*(1-a*dt/2+(a*dt)^2/6); g=dt^2*(.5-a*dt/6+(a*dt)^2/24);
else
    h=-expm1(-a*dt)/a; g=(dt-h)/a;
end
v=[0;filter(h,[1 -r],u(1:end-1))];
p=[0;cumsum(h*v(1:end-1)+g*u(1:end-1))];
end
function [r,b]=metrica(y,p)
e=p-y; r=sqrt(mean(e.^2)); b=mean(e);
end
