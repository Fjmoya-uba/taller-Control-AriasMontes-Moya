function resumen = probar_variantes_corregidas
% Comparacion temporal continua; no modifica mediciones ni informe.
d=fileparts(mfilename('fullpath')); destino=fullfile(d,'Variantes');
if ~isfolder(destino), mkdir(destino); end
S=load(fullfile(d,'..','mediciones_dinamica_carrito_1_corregida.mat'));
O=load(fullfile(d,'..','..','Mediciones','mediciones_dinamica_carrito_1.mat'));
t=double(S.out.tout(:)); y=double(S.out.d2(:))-15.612;
u=double(S.out.d3(:)); ang=double(S.out.d1(:));
ok=isfinite(t)&isfinite(y)&isfinite(u)&isfinite(ang)&y>=-15&y<=21&t<=31.8;
f=find(~ok,1); if isempty(f), f=numel(t)+1; end
n=f-1; t=t(1:n); y=y(1:n); u=u(1:n); ang=ang(1:n);
assert(n>=100 && abs(t(1))<1e-9 && max(abs(diff(t)-median(diff(t))))<1e-6);
ne=floor(.75*n); train=(1:ne)'; val=(ne+1:n)'; y0=.2995;
activo=abs(u-median(u(t<1)))>1;
ini=find(diff([false;activo])==1); fin=find(diff([activo;false])==-1);
% Categorias: pulso positivo/negativo, descanso positivo/negativo.
grupo=zeros(n,1); episodio=zeros(n,1); tramos={}; signos=[];
for j=1:numel(ini)
    signo=sign(median(double(O.out.d1(ini(j):fin(j)))));
    assert(signo~=0,'Pulso sin signo definido.');
    ultimo=n; if j<numel(ini), ultimo=ini(j+1)-1; end
    grupo(ini(j):fin(j))=1+(signo<0);
    grupo(fin(j)+1:ultimo)=3+(signo<0);
    episodio(ini(j):ultimo)=j;
    tramos{j}=(fin(j)+1:ultimo)'; signos(j)=signo; %#ok<AGROW>
end
% Copia alternativa: cero exacto desde el retorno del comando al centro.
% Incluye ambos signos e inicio en reposo; conserva los pulsos originales.
Z=O; cero=double(O.out.d1(:)); comando=double(O.out.d3(:));
reposo=abs(comando-median(comando(double(O.out.tout(:))<1)))<=1;
cero(reposo)=0; Z.out.d1=reshape(cast(cero,'like',O.out.d1),size(O.out.d1));
control=Z; control.out.d1=O.out.d1; assert(isequaln(control,O));
assert(isequaln(Z.out.d1(~reposo),O.out.d1(~reposo)));
save(fullfile(destino,'mediciones_descansos_cero.mat'),'-struct','Z');
nombres=["Residual_base";"Residual_c0";"Residual_v0_libre";"Residual_equilibrado"; ...
    "Cero_base";"Cero_c0";"Cero_v0_libre";"Cero_equilibrado"];
grilla=unique([0 logspace(-3,2,100)]); perfiles=zeros(numel(grilla),8);
pred=zeros(n,8); filas={}; detalle={}; pendientes={};
for k=1:8
    tipo=mod(k-1,4)+1; entrada=ang; if k>4, entrada=cero(1:n); end
    pesos=ones(ne,1);
    if tipo==4
        assert(all(arrayfun(@(g)nnz(grupo(train)==g)>0,1:4)));
        for g=1:4, idx=grupo(train)==g; pesos(idx)=1/nnz(idx); end
        % El reposo inicial constituye una quinta categoria de igual peso.
        idx=grupo(train)==0; if any(idx), pesos(idx)=1/nnz(idx); end
        pesos=pesos/mean(pesos);
    end
    obj=@(a) ajustar(a,t,entrada,y,y0,train,pesos,tipo);
    for j=1:numel(grilla), perfiles(j,k)=obj(grilla(j)); end
    [~,j]=min(perfiles(:,k)); a=grilla(j);
    if j>1 && j<numel(grilla)
        candidato=fminbnd(obj,grilla(j-1),grilla(j+1));
        if obj(candidato)<obj(a), a=candidato; end
    end
    [ecm,coef,B]=obj(a); pred(:,k)=y0+B*coef;
    K=coef(1); c=0; v0=0;
    if tipo~=2, c=coef(2); end
    if tipo==3, v0=coef(3); end
    [rt,ft]=metricas(y(train),pred(train,k)); [rv,fv]=metricas(y(val),pred(val,k));
    filas(end+1,:)={nombres(k),a,K,c,v0,rt,ft,rv,fv,ecm,a==0||a==100}; %#ok<AGROW>
    for parte=1:2
        idx=train; if parte==2, idx=val; end
        for g=0:4
            ii=idx(grupo(idx)==g); if isempty(ii), continue; end
            [r,fm]=metricas(y(ii),pred(ii,k));
            detalle(end+1,:)={nombres(k),parte,g,numel(ii),r,fm,mean(pred(ii,k)-y(ii))}; %#ok<AGROW>
        end
    end
    for j=1:numel(tramos)
        idx=tramos{j}; if isempty(idx), continue; end
        idx=idx(t(idx)>=t(idx(1))+.8); if numel(idx)<10, continue; end
        pr=polyfit(t(idx)-t(idx(1)),y(idx),1); ps=polyfit(t(idx)-t(idx(1)),pred(idx,k),1);
        pendientes(end+1,:)={nombres(k),j,signos(j),t(idx(1)),t(idx(end)),all(idx>ne),pr(1),ps(1),ps(1)-pr(1)}; %#ok<AGROW>
    end
end
resumen=cell2table(filas,'VariableNames',{'modelo','a','K','c','v0','RMSE_train_cm','FIT_train_pct','RMSE_val_cm','FIT_val_pct','objetivo_ponderado','limite_busqueda'});
D=cell2table(detalle,'VariableNames',{'modelo','particion_1_train_2_val','grupo','muestras','RMSE_cm','FIT_pct','sesgo_cm'});
P=cell2table(pendientes,'VariableNames',{'modelo','episodio','signo','inicio_s','fin_s','solo_validacion','pendiente_real','pendiente_sim','error_cm_s'});
writetable(resumen,fullfile(destino,'resumen.csv')); writetable(D,fullfile(destino,'por_tramos.csv'));
writetable(P,fullfile(destino,'pendientes.csv'));
save(fullfile(destino,'resultados.mat'),'resumen','D','P','t','y','ang','cero','pred','grilla','perfiles','grupo','episodio','ne','reposo');
fig=figure('Visible','off','Color','w','Position',[100 100 1400 950]);
for bloque=1:2
    subplot(3,2,bloque); plot(t,ang,t,cero(1:n)); grid on; ylabel('Angulo [grados]'); legend('Residual','Cero');
    subplot(3,2,bloque+2); ids=(1:4)+4*(bloque-1); plot(t,y,'k',t,pred(:,ids)); xline(t(ne+1),':k'); grid on;
    ylabel('Posicion [cm]'); legend(['Real';nombres(ids)],'Interpreter','none');
    subplot(3,2,bloque+4); plot(t,pred(:,ids)-y); xline(t(ne+1),':k'); grid on; ylabel('Error [cm]'); xlabel('Tiempo [s]');
end
exportgraphics(fig,fullfile(destino,'comparacion.png'),'Resolution',150); close(fig); disp(resumen);
end
function [ecm,coef,B]=ajustar(a,t,u,y,y0,idx,pesos,tipo)
sys=ss([0 1;0 -a],[0;1],[1 0],0);
B=lsim(sys,u,t,[0;0],'zoh');
if tipo~=2, B=[B lsim(sys,ones(size(u)),t,[0;0],'zoh')]; end
if tipo==3
    rv=t; if a>1e-10, rv=-expm1(-a*t)/a; end
    B=[B rv];
end
W=sqrt(pesos); X=B(idx,:).*W; z=(y(idx)-y0).*W;
coef=X\z; ecm=mean((X*coef-z).^2);
end
function [r,f]=metricas(y,p)
r=sqrt(mean((p-y).^2)); den=norm(y-mean(y)); f=NaN;
if den>eps, f=100*(1-norm(p-y)/den); end
end
