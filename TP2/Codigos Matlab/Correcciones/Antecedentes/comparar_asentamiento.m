function resumen = comparar_asentamiento
% Identificacion libre/restringida con igual particion y condiciones.
% Ejecutar primero corregir_angulo_tras_asentamiento. No disena observador.
d=fileparts(mfilename('fullpath')); salida=fullfile(d,'Asentamiento');
O=load(fullfile(d,'..','..','Mediciones','mediciones_dinamica_carrito_1.mat'));
S=load(fullfile(salida,'mediciones_angulo_cero_tras_asentamiento.mat'));
t=double(O.out.tout(:)); y=double(O.out.d2(:))-15.612;
u=double(O.out.d3(:)); ang=double(O.out.d1(:)); cero=double(S.out.d1(:));
ok=isfinite(t)&isfinite(y)&isfinite(u)&isfinite(ang)&y>=-15&y<=21&t<=31.8;
f=find(~ok,1); if isempty(f), f=numel(t)+1; end
n=f-1; t=t(1:n); y=y(1:n); ang=ang(1:n); cero=cero(1:n);
assert(n>=100 && abs(t(1))<1e-9 && max(abs(diff(t)-median(diff(t))))<1e-6);
ne=floor(.75*n); train=(1:ne)'; val=(ne+1:n)'; y0=.2995;
nombres=["Original_libre";"Original_a_2_5";"Asentamiento_libre";"Asentamiento_a_2_5"];
pred=zeros(n,4); filas={}; grillas=cell(4,1); perfiles=grillas;
for k=1:4
    entrada=ang; if k>2, entrada=cero; end
    limites=[0 100]; grilla=unique([0 logspace(-3,2,150) linspace(2,5,61)]);
    if mod(k,2)==0, limites=[2 5]; grilla=linspace(2,5,121); end
    obj=@(a) ajustar(a,t,entrada,y,y0,train);
    perfil=arrayfun(obj,grilla); [~,j]=min(perfil); a=grilla(j);
    % Refinar todos los minimos interiores detectados, conservar extremos.
    candidatos=find(perfil(2:end-1)<=perfil(1:end-2) & perfil(2:end-1)<=perfil(3:end))+1;
    for j=candidatos
        aa=fminbnd(obj,grilla(j-1),grilla(j+1));
        if obj(aa)<obj(a), a=aa; end
    end
    [ecm,coef,B]=obj(a); pred(:,k)=y0+B*coef;
    [rt,ft]=metricas(y(train),pred(train,k)); [rv,fv]=metricas(y(val),pred(val,k));
    borde=min(abs(a-limites))<1e-5;
    filas(end+1,:)={nombres(k),a,-a,coef(1),coef(2),rt,ft,rv,fv,ecm,borde}; %#ok<AGROW>
    grillas{k}=grilla; perfiles{k}=perfil;
end
resumen=cell2table(filas,'VariableNames',{'modelo','a','polo','K','c', ...
    'RMSE_train_cm','FIT_train_pct','RMSE_val_cm','FIT_val_pct','ECM_train','en_limite'});
writetable(resumen,fullfile(salida,'identificacion.csv'));
save(fullfile(salida,'identificacion.mat'),'resumen','pred','t','y','ang','cero', ...
    'ne','y0','grillas','perfiles','nombres');
f=figure('Visible','off','Color','w','Position',[100 100 1300 950]);
subplot(3,1,1); plot(t,y,'k',t,pred); xline(t(ne+1),':k'); grid on;
ylabel('Posicion [cm]'); legend(['Medida';nombres],'Interpreter','none');
subplot(3,1,2); plot(t,pred-y); xline(t(ne+1),':k'); grid on;
ylabel('Error [cm]'); xlabel('Tiempo [s]');
subplot(3,1,3); hold on;
for k=1:4, plot(grillas{k},perfiles{k}); end
xlim([0 12]); xline(2,':k'); xline(5,':k'); grid on;
xlabel('a [1/s]'); ylabel('ECM entrenamiento');
legend(nombres,'Interpreter','none');
exportgraphics(f,fullfile(salida,'identificacion.png'),'Resolution',150); close(f);
disp(resumen);
end
function [ecm,coef,B]=ajustar(a,t,u,y,y0,idx)
sys=ss([0 1;0 -a],[0;1],[1 0],0);
B=[lsim(sys,u,t,[0;0],'zoh') lsim(sys,ones(size(u)),t,[0;0],'zoh')];
coef=B(idx,:)\(y(idx)-y0); ecm=mean((B(idx,:)*coef-(y(idx)-y0)).^2);
end
function [r,f]=metricas(y,p)
r=sqrt(mean((p-y).^2)); den=norm(y-mean(y)); f=NaN;
if den>eps, f=100*(1-norm(p-y)/den); end
end
