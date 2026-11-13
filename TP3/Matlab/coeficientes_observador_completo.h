// Generado por observador_completo.m. No editar a mano.
// Estados: theta [rad], omega [rad/s], p [cm], v [cm/s].
// Salidas: theta [rad], p [cm]. Entrada relativa [us].
// x[k]=AD*x[k-1]+BU*(u[k-d]+u[k-d-1])+BY*(y[k]+y[k-1]).
// Retardo heredado de P7: confirmar contra TP2 antes del ensayo.
#pragma once
const float OBS_TS = 2.000000000000e-02f;
const unsigned int OBS_DELAY_SAMPLES = 2;
const float OBS_AD[4][4] = {
  {5.585866479808e-01f, 1.038227183574e-02f, 7.183246446980e-18f, 1.952945127773e-17f},
  {-3.164271715443e+01f, 1.214846978788e-01f, -1.458356106584e-16f, -3.964905664776e-16f},
  {1.113490582832e-01f, 7.417336682864e-04f, 9.093837619918e-01f, 1.848954079636e-02f},
  {1.161528554957e+01f, 7.737333832649e-02f, -8.242012891068e-01f, 9.287212603457e-01f},
};
const float OBS_BU[4] = {3.335980871894e-05f, 3.603499849973e-03f, 2.383302391415e-06f, 2.486122312492e-04f};
const float OBS_BY[4][2] = {
  {1.704824360042e-01f, -7.183246446980e-18f},
  {1.039617635123e+01f, 1.458356106584e-16f},
  {1.217966208518e-02f, 4.530811900408e-02f},
  {1.270511445701e+00f, 4.121006445534e-01f},
};
