function graficar_variantes_asentamiento
d=fileparts(mfilename('fullpath')); salida=fullfile(d,'Asentamiento','Variantes');
D=load(fullfile(salida,'evaluacion.mat')); colores=lines(4);
fuentes=["Original","Constantes","Cero_inmediato","Asentamiento"];
etiquetas=["Base";"c=0";"v0 libre";"Equilibrado"];
for fuente=1:4
    ids=(fuente-1)*8+(1:8);
    f=figure('Visible','off','Color','w','Position',[100 100 1400 1000]);
    for rango=1:2
        kk=ids(rango:2:8);
        subplot(3,2,rango); plot(D.t,D.y,'k'); hold on;
        for j=1:4, plot(D.t,D.pred(:,kk(j)),'Color',colores(j,:)); end
        xline(D.t(D.ne+1),':k','HandleVisibility','off'); grid on; ylabel('Posicion [cm]');
        legend(['Medida';etiquetas],'Location','best');
        titulo='Libre: a en [0,100]'; if rango==2, titulo='Restringida: a en [2,5]'; end
        title(titulo);
        subplot(3,2,rango+2); hold on;
        for j=1:4, plot(D.t,D.pred(:,kk(j))-D.y,'Color',colores(j,:)); end
        xline(D.t(D.ne+1),':k'); grid on; ylabel('Error [cm]');
        subplot(3,2,rango+4); plot(D.t,D.vref(:,1),'Color',[.65 .65 .65]); hold on;
        plot(D.t,D.vref(:,2),'--k');
        for j=1:4, plot(D.t,D.velocidad(:,kk(j)),'Color',colores(j,:)); end
        xline(D.t(D.ne+1),':k','HandleVisibility','off'); grid on;
        ylabel('Velocidad [cm/s]'); xlabel('Tiempo [s]');
        legend(['Derivada 0.20 s';'Derivada 0.40 s';etiquetas],'Location','best');
    end
    sgtitle(strrep(fuentes(fuente),'_',' '));
    exportgraphics(f,fullfile(salida,fuentes(fuente)+".png"),'Resolution',150); close(f);
end
end
