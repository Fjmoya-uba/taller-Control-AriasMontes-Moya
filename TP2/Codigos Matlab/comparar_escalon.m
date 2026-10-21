function resultado = comparar_escalon(archivo, C, nombre)
% Comparador de escalon para P, PI y PD filtrado (nombre historico).
% Sin argumentos conserva el ensayo P original.
% PI: comparar_escalon_P('controlador_PI.mat', ...
%         zpk(-0.5,0,db2mag(22)), 'PI')
% C debe ser el controlador CONTINUO usado en el ensayo, no Cz.
s = load(archivo);
o = s.out;
% Protocolo del sketch: referencia absoluta, distancia absoluta, pulso us.
t = double(o.tout(:));
r = double(o.d1(:)) - 15.612;
y = double(o.d2(:)) - 15.612;
u = double(o.d3(:));
assert(numel(t)==numel(r) && numel(t)==numel(y) && numel(t)==numel(u));
assert(all(isfinite(t)) && all(diff(t)>0), 'El tiempo debe ser finito y creciente.');
assert(all(isfinite(r)) && all(isfinite(u)), 'Referencia o pulso invalidos.');
inicio = find(abs(diff(r)) > 5, 1) + 1;
assert(~isempty(inicio), 'No se detecto el escalon de referencia.');
t = t - t(inicio); % Solo alineacion temporal: no corregir posiciones.
valida = isfinite(y) & double(o.d2(:)) > 0;
y(~valida) = NaN;
fprintf('Escalon real: %.4f -> %.4f cm; muestras invalidas: %d\n', ...
    r(inicio-1), r(inicio), sum(~valida));

% Tustin del controlador y ZOH de la planta, igual que disenar_control.m
Ts = 0.02;
C=tf(C);
assert(C.Ts==0 && isproper(C), 'Pasar un controlador continuo propio (PD filtrado).');
Cz=c2d(C,Ts,'tustin');
[b,a]=tfdata(Cz,'v'); b=b/a(1); a=a/a(1);
assert(numel(a)<=3 && numel(b)<=3, 'Arduino admite orden hasta dos.');
b(end+1:3)=0; a(end+1:3)=0;
fprintf('Arduino: A1=%.9g A2=%.9g B0=%.9g B1=%.9g B2=%.9g\n', ...
    -a(2),-a(3),b(1),b(2),b(3));
% Misma dinamica nominal de carrito y servo que en disenar_control.m.
% Estados: posicion, velocidad, angulo y velocidad angular; sesgo c=0.
planta = ss([0 1 0 0; 0 -6.815007 24.428153 0; ...
    0 0 0 1; 0 0 -967.5 -50.12], [0;0;0;36.82], [1 0 0 0], 0);
pd = c2d(planta, Ts, 'zoh');
assert(isstable(feedback(Cz*tf(pd),1)), 'El lazo digital nominal es inestable.');
ts = (0:Ts:ceil(max(t)/Ts)*Ts)';
rs = repmat(r(inicio), size(ts));
x = zeros(4,1); % Simulacion nominal: reposo en cero, c=0.
ys = zeros(size(ts)); us = zeros(size(ts));
errores=zeros(2,1); salidas=zeros(2,1);
saturadas=0;
for k=1:numel(ts)
    ys(k) = x(1);
    e=rs(k)-ys(k);
    salida=-a(2:3)*salidas + b*[e;errores];
    pedido=1544+salida;
    limitado=min(max(pedido,1100),2000);
    saturadas=saturadas+(pedido~=limitado);
    us(k)=round(limitado);
    errores=[e;errores(1)];
    % Memoria limitada sin redondear, como en control_carrito_tp2.ino.
    salidas=[limitado-1544;salidas(1)];
    x = pd.A*x + pd.B*(us(k)-1544);
end
assert(abs(r(inicio-1))<0.1 && abs(r(inicio)-10)<0.1, ...
    'Este script espera el ensayo de 0 a 10 cm. Revisar los canales.');
assert(all(abs(r(inicio:end)-r(inicio))<0.1) && ...
    all(abs(r(1:inicio-1))<0.1), 'Se espera un solo escalon de 0 a 10 cm.');

%% Metricas respecto de la referencia, sin eliminar el error final
real = metricas(t(inicio:end), y(inicio:end), r(inicio), u(inicio:end));
simulada = metricas(ts, ys, rs(1), us);
comparacion = array2table([real; simulada], ...
    'RowNames', {'Real','Simulada'}, 'VariableNames', ...
    {'Establecimiento_2pct_s','Sobrepaso_pct','Error_final_2s_cm', ...
     'Pulso_min_us','Pulso_max_us'});
disp(comparacion);
% RMSE de posicion sobre muestras reales validas posteriores al escalon.
post = t>=0 & valida;
prediccion = interp1(ts, ys, t(post), 'linear', NaN);
rmse_cm = sqrt(mean((y(post)-prediccion).^2, 'omitnan'));
fprintf('RMSE real-simulacion: %.4f cm\n', rmse_cm);
fprintf('Muestras saturadas en simulacion: %d\n',saturadas);

figure('Color','white','Name',[char(nombre) ': escalon real vs simulado']);
subplot(2,1,1);
plot(t,r,':k',t,y,'b',ts,ys,'--r'); grid on; hold on;
yline(10.2, ':', 'Banda 2 %'); yline(9.8, ':');
ylabel('Posicion [cm]'); legend('Referencia','Real','Simulada','Location','best');
subplot(2,1,2);
stairs(t,u,'b'); hold on; stairs(ts,us,'--r'); grid on;
yline(1100,':'); yline(2000,':');
xlabel('Tiempo desde el escalon [s]'); ylabel('Pulso [us]');
legend('Real','Simulado','Location','best');

resultado=struct('comparacion',comparacion,'rmse_cm',rmse_cm, ...
    't',t,'r',r,'y',y,'u',u,'ts',ts,'ys',ys,'us',us, ...
    'C',C,'Cz',Cz,'saturadas',saturadas);
end

function m = metricas(t,y,objetivo,u)
    validas = isfinite(y);
    assert(any(validas),'Sin posiciones validas para calcular metricas.');
    sobrepaso = max(0,(max(y(validas))-objetivo)/abs(objetivo)*100);
    final = t >= t(end)-2 & validas;
    errorFinal = mean(objetivo-y(final));
    % NaN: no se establecio durante el registro o hubo lecturas invalidas
    % en la cola. El tiempo obtenido solo garantiza permanencia hasta el fin.
    fuera = ~validas | abs(y-objetivo)>0.02*abs(objetivo);
    ultimo = find(fuera,1,'last');
    establecimiento = NaN;
    if isempty(ultimo)
        establecimiento = t(1);
    elseif ultimo < numel(t)
        establecimiento = t(ultimo+1);
    end
    m = [establecimiento,sobrepaso,errorFinal,min(u),max(u)];
end
