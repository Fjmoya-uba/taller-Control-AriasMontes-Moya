clear; clc;

%% 1. Modelo ampliado
% Estados: x = [angulo; velocidad angular; sesgo]
% Suponemos que el sesgo es constante: derivada del sesgo = 0.

A = [    0       1    0;
      -967.5  -50.12  0;
         0       0    0];

% Entrada: variacion del pulso respecto de 1544 us.
B = [0; 36.82; 0];

%% 2. Mediciones
% Primera medicion: angulo.
% Segunda medicion: velocidad del giroscopo = velocidad real + sesgo.

C = [1 0 0;
     0 1 1];

% Verificar que se pueden estimar los tres estados.
assert(rank(obsv(A,C)) == 3, 'El modelo no es observable');

%% 3. Polos del observador
% Conservamos el par continuo del observador Tustin que funciona.

A_barra = A(1:2, 1:2);

omega_n = 2 * max(abs(real(eig(A_barra))));
zeta = 0.58;

p1 = omega_n * (-zeta + 1.25*1i*sqrt(1-zeta^2));
p2 = conj(p1);

% Polo adicional para estimar el sesgo.
% -5 es una eleccion inicial que despues debemos validar.
p3 = -9;

polos = [p1 p2 p3];

%% 4. Conservar el observador original y agregar la estimacion de sesgo
% Misma ganancia de exportar_observador_tustin.m para [angulo; velocidad].
% Ambas coordenadas angulares se expresan aqui en grados en lugar de radianes;
% la ganancia original no cambia con esta conversion comun de unidades.
L_original = real(acker(A_barra', [1 0]', [p1 p2])');

% Solo el error angular corrige los dos primeros estados.
% Solo el error del giroscopo corrige el sesgo:
% sesgo_hat_dot = (-p3)*(gyro - velocidad_hat - sesgo_hat).
L = [L_original(1), 0;
     L_original(2), 0;
                 0, -p3];

% Ecuacion continua:
% xhat_dot = A*xhat + B*u + L*(y - C*xhat)

% Agrupando:
% xhat_dot = (A-L*C)*xhat + B*u + L*y

F = A - L*C;
% F es triangular por bloques: conserva el par original y agrega p3.
assert(max(abs(poly(F)-poly(polos))) < 1e-7, 'Polos inconsistentes');

%% 5. Discretizacion del observador completo por Tustin
Ts = 0.02;
h = Ts/2;

I = eye(3);
M = I - h*F;

Ad = M \ (I + h*F);
Bu = M \ (h*B);
By = M \ (h*L);

%% 6. Mostrar los coeficientes para Arduino
disp('Ganancia continua L:');
disp(L);

disp('Ad:');
disp(Ad);

disp('Bu:');
disp(Bu);

disp('By:');
disp(By);

disp('Polos discretos del observador:');
disp(eig(Ad));
