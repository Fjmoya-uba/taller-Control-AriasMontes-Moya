function diagnosticar_variantes_asentamiento
% Complemento reproducible sobre curvas guardadas; no vuelve a ajustar.
d=fileparts(mfilename('fullpath')); salida=fullfile(d,'Asentamiento','Variantes');
D=load(fullfile(salida,'evaluacion.mat')); R=D.R; t=D.t;
% Verificacion independiente de las curvas guardadas contra lsim.
for k=1:height(R)
    fuente=find(["Original","Constantes","Cero_inmediato","Asentamiento"]==R.entrada(k));
    a=R.a(k); sys=ss([0 1;0 -a],[0;1],eye(2),zeros(2,1));
    x=lsim(sys,R.K(k)*D.entradas(:,fuente)+R.c(k),t,[0;R.v0(k)],'zoh');
    assert(max(abs(x(:,1)+D.y0-D.pred(:,k)))<1e-6);
    assert(max(abs(x(:,2)-D.velocidad(:,k)))<1e-6);
    if R.ajuste(k)=="equilibrado"
        totales=arrayfun(@(g)sum(D.pesos_guardados(D.grupo(1:D.ne)==g,k)),0:4);
        assert(max(totales)-min(totales)<1e-8);
    end
end
assert(abs(R.RMSE_val(R.modelo=="Original_base_libre")-1.1523857)<1e-5);
assert(abs(R.RMSE_val(R.modelo=="Constantes_base_libre")-2.7054603)<1e-5);
assert(abs(R.RMSE_val(R.modelo=="Constantes_v0_libre_libre")-1.8189972)<1e-5);
filas={};
for k=1:height(R)
 for j=unique(D.episodio(D.episodio>0))'
    descanso=find(D.episodio==j & D.grupo>=3); if isempty(descanso), continue; end
    tarde=descanso(t(descanso)>=t(descanso(1))+.8);
    for parte=1:2
        ii=tarde(tarde<=D.ne); if parte==2, ii=tarde(tarde>D.ne); end
        if numel(ii)<10, continue; end
        pr=polyfit(t(ii)-t(ii(1)),D.y(ii),1); ps=polyfit(t(ii)-t(ii(1)),D.pred(ii,k),1);
        filas(end+1,:)={R.modelo(k),j,parte,D.grupo(ii(1)),t(ii(1)),t(ii(end)), ...
            pr(1),ps(1),ps(1)-pr(1)}; %#ok<AGROW>
    end
 end
end
T=cell2table(filas,'VariableNames',{'modelo','episodio','particion','grupo','inicio_s','fin_s', ...
    'pendiente_real','pendiente_sim','error_cm_s'});
writetable(T,fullfile(salida,'pendientes_tardias.csv'));
filas={};
for k=1:height(R)
    ii=T.modelo==R.modelo(k)&T.particion==2;
    mae=mean(abs(T.error_cm_s(ii))); peor=max(abs(T.error_cm_s(ii)));
    v=D.V; jj=v.modelo==R.modelo(k)&v.particion==2&v.grupo==-1;
    filas(end+1,:)={R.modelo(k),mae,peor,v.RMSE_cm_s(jj&v.ventana_muestras==11), ...
        v.RMSE_cm_s(jj&v.ventana_muestras==21)}; %#ok<AGROW>
end
M=cell2table(filas,'VariableNames',{'modelo','MAE_pendiente_tardia_val','max_error_pendiente_val', ...
    'RMSE_vel_11_val','RMSE_vel_21_val'});
writetable(M,fullfile(salida,'diagnostico.csv'));
ii=t<=1; v0ref=median(D.vref(ii,:),1,'omitnan');
save(fullfile(salida,'diagnostico.mat'),'T','M','v0ref');
fprintf('Verificadas 32 curvas contra lsim, pesos y reproduccion de antecedentes.\n');
fprintf('Referencia derivada mediana primer segundo (11/21): %.6f / %.6f cm/s.\n',v0ref);
disp(T(T.particion==2 & (startsWith(T.modelo,"Asentamiento")|T.modelo=="Original_base_libre"),:));
end
