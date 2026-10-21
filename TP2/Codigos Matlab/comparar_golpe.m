function resultado = comparar_golpe(archivo, C, nombre, instanteGolpe, saltoVelocidad)
% Comparador reutilizable P/PI/PD. C es el controlador CONTINUO utilizado.
% Ejemplo P (deteccion orientativa del comienzo del movimiento):
%   comparar_golpe('controlador_proporcional_golpe.mat',19.95262315,'P')
% Para fijar el instante real y el salto de velocidad simulado:
%   comparar_golpe('controlador_proporcional_golpe.mat',19.95262315,'P',5.2,5)
% PI/PD: pasar su transferencia continua, con filtro derivativo si corresponde.
% Se aplica Tustin a Ts=0.02 y la misma recurrencia limitada del Arduino.
if nargin<1, archivo='controlador_proporcional_golpe.mat'; end
if nargin<2, C=19.95262315; end
if nargin<3, nombre='P'; end
if nargin<4, instanteGolpe=[]; end
if nargin<5, saltoVelocidad=[]; end
carpeta=fileparts(mfilename('fullpath'));
if ~isfile(archivo), archivo=fullfile(carpeta,archivo); end
s=load(archivo); o=s.out;
t=double(o.tout(:)); r=double(o.d1(:))-15.612;
y=double(o.d2(:))-15.612; u=double(o.d3(:));
assert(numel(t)==numel(r) && numel(t)==numel(y) && numel(t)==numel(u));
assert(all(isfinite(t)) && all(diff(t)>0),'Tiempo invalido.');
assert(max(abs(r))<0.1,'Se espera referencia fija en cero para el golpe.');
valida=isfinite(y) & double(o.d2(:))>0;
y(~valida)=NaN;
% La primera parte del registro debe contener al menos 1 s sin golpes.
base=t<=t(1)+1 & valida;
assert(nnz(base)>=10,'No hay suficientes muestras iniciales validas.');
yBase=median(y(base));
ruido=1.4826*median(abs(y(base)-yBase));
umbral=max(0.4,6*ruido);
automatico=isempty(instanteGolpe);
if automatico
    fuera=t>t(1)+1 & valida & abs(y-yBase)>umbral;
    % Tres lecturas consecutivas reducen disparos por ecos aislados.
    j=find(conv(double(fuera),ones(3,1),'valid')==3,1);
    assert(~isempty(j),'No se detecta el golpe. Indicar su instante como cuarto argumento.');
    instanteGolpe=t(j);
    fprintf('Inicio estimado por movimiento: %.3f s (umbral %.3f cm).\n',instanteGolpe,umbral);
    fprintf('Revisar la figura; puede ser posterior al contacto. Ajustar el cuarto argumento.\n');
end
assert(isscalar(instanteGolpe) && isfinite(instanteGolpe) && ...
    instanteGolpe>t(1) && instanteGolpe<t(end),'Instante fuera del registro.');
if isempty(saltoVelocidad)
    % Solo elegir el sentido; no ajustar amplitud para hacer coincidir curvas.
    ventana=t>=instanteGolpe & t<=instanteGolpe+0.3 & valida;
    sentido=sign(median(y(ventana))-yBase);
    if sentido==0, sentido=1; end
    saltoVelocidad=5*sentido;
end
assert(isscalar(saltoVelocidad) && isfinite(saltoVelocidad));
ta=t-instanteGolpe;

Ts=0.02;
C=tf(C);
assert(C.Ts==0 && isproper(C),'Pasar un controlador continuo propio (PD filtrado).');
Cz=c2d(C,Ts,'tustin');
[b,a]=tfdata(Cz,'v'); b=b/a(1); a=a/a(1);
assert(numel(a)<=3 && numel(b)<=3,'La recurrencia Arduino admite orden hasta dos.');
a(end+1:3)=0; b(end+1:3)=0;
fprintf('Arduino: A1=%.9g A2=%.9g B0=%.9g B1=%.9g B2=%.9g\n', ...
    -a(2),-a(3),b(1),b(2),b(3));
% Misma dinamica nominal de carrito y servo que en disenar_control.m.
% Estados: posicion, velocidad, angulo y velocidad angular; sesgo c=0.
planta=ss([0 1 0 0;0 -6.815007 24.428153 0; ...
    0 0 0 1;0 0 -967.5 -50.12],[0;0;0;36.82],[1 0 0 0],0);
pd=c2d(planta,Ts,'zoh');
assert(isstable(feedback(Cz*tf(pd),1)),'El lazo digital nominal es inestable.');
ts=(0:Ts:ceil(max(ta)/Ts)*Ts)';
x=[0;saltoVelocidad;0;0]; memoriaE=[0;0]; memoriaU=[0;0];
ys=zeros(size(ts)); us=ys; saturadas=0;
for k=1:numel(ts)
    ys(k)=x(1); e=-ys(k);
    salida=-a(2:3)*memoriaU + b*[e;memoriaE];
    pedido=1544+salida;
    limitado=min(max(pedido,1100),2000);
    saturadas=saturadas+(pedido~=limitado);
    us(k)=round(limitado);
    memoriaE=[e;memoriaE(1)];
    memoriaU=[limitado-1544;memoriaU(1)];
    x=pd.A*x+pd.B*(us(k)-1544);
end
post=ta>=0;
real=metricas(ta(post),y(post),u(post));
simulada=metricas(ts,ys,us);
comparacion=array2table([real;simulada],'RowNames',{'Real','Simulada'}, ...
    'VariableNames',{'Max_abs_cm','Recuperacion_banda_0p2cm_s', ...
    'Error_final_2s_cm','Pulso_min_us','Pulso_max_us','Muestras_en_limite'});
disp(comparacion);
fprintf('Salto simulado: %.2f cm/s; saturaciones simuladas: %d.\n',saltoVelocidad,saturadas);
fprintf('Posicion inicial mediana real: %.4f cm. No se resta de las curvas.\n',yBase);
fprintf('La amplitud del impulso real no esta medida; la comparacion es cualitativa.\n');
figure('Color','white','Name',['Golpe: ' char(nombre)]);
subplot(2,1,1);
plot(ta,r,':k',ta,y,'b',ts,ys,'--r'); hold on; grid on;
xline(0,':'); yline(0.2,':'); yline(-0.2,':');
ylabel('Posicion [cm]'); legend('Referencia','Real','Simulada','Location','best');
subplot(2,1,2);
stairs(ta,u,'b'); hold on; stairs(ts,us,'--r'); grid on;
yline(1100,':'); yline(2000,':');
xlabel('Tiempo desde el inicio seleccionado [s]'); ylabel('Pulso [us]');
legend('Real','Simulado','Location','best');
resultado=struct('comparacion',comparacion,'instanteGolpe',instanteGolpe, ...
    'inicioAutomatico',automatico,'saltoVelocidad',saltoVelocidad, ...
    't',ta,'r',r,'y',y,'u',u,'ts',ts,'ys',ys,'us',us,'Cz',Cz);
end

function m=metricas(t,y,u)
validas=isfinite(y);
assert(any(validas),'Sin posiciones validas.');
fuera=~validas | abs(y)>0.2;
ultimo=find(fuera,1,'last'); recuperacion=NaN;
if isempty(ultimo), recuperacion=t(1);
elseif ultimo<numel(t), recuperacion=t(ultimo+1); end
final=t>=t(end)-2 & validas;
m=[max(abs(y(validas))),recuperacion,mean(-y(final)),min(u),max(u), ...
    sum(u<=1100 | u>=2000)];
end
