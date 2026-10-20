% 1. Extraer los datos del objeto SimulationOutput a una estructura estándar
out_struct.d1 = out.d1;
out_struct.d2 = out.d2;
out_struct.d3 = out.d3
out_struct.tout = out.tout;

% 2. Reemplazar la variable out y guardarla en el archivo .mat
out = out_struct;
save('control_proporcional_integral_derivativo.mat', 'out');