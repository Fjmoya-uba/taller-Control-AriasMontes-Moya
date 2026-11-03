clear variables;

% modelo servo-barra con retardo
G = tf(36.82, [1 50.12 967.5]);

[num, den] = tfdata(G, 'v');

p_cont = pole(G);

Ac = [0, 1, 0; -den(end), -den(2), 0; 0, 0, 0];
Bc = [0; num(end); 0];
C = [1 0 0; 0 1 1];

% discretizo matrices continuas
Ts = 0.02;
%Ad = eye(3) + Ac*Ts;
%Bd = Bc*Ts;
%Cd = C;

sysd = c2d(ss(Ac, Bc, C, 0), Ts, 'zoh');
Ad = sysd.A;  Bd = sysd.B;  Cd = C;

p_obs = [2*p_cont(1), 2*p_cont(2), -5];
p_obs_disc = exp(Ts*p_obs);

Ld = (place(Ad', Cd', p_obs_disc))';