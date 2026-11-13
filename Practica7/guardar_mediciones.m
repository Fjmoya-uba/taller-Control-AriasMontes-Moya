% 1. Extraer los datos del objeto SimulationOutput a una estructura estándar
out_struct.d1 = out.d1;
out_struct.d2 = out.d2;
out_struct.d3 = out.d3;
out_struct.d4 = out.d4;
out_struct.d5 = out.d5;
out_struct.d6 = out.d6;
out_struct.tout = out.tout;

% 2. Reemplazar la variable out y guardarla en el archivo .mat
out = out_struct;
save('sesgo_tustin_ensayo_04.mat', 'out');