clear, clc, close all
%% Configuracion
% Periodo de muestreo
Tm = 0.01; 

% Tiempo final de simulacion
Tend = 0.3; % seg
t = 0:Tm:Tend-Tm;
Lt = length(t);

% Senyal de referencia: escalon
ini_zeros = 5;
r = [zeros(1,ini_zeros) ones(1,Lt-ini_zeros)];

%% Sección: Modelo lineal
Tm = 0.01; % Periodo de muestreo
s = tf([1 0],1);

% Polos y ganancia
a = 1.77; % Polo dominante del motor
b = a*80; % 2º polo del motor (no dominante)
k = 3.6; % Ganancia del motor 

G = k*a/(s+a)*b/(s+b);
Gz = c2d(G,Tm);

%Cp = Vp / E = Kp
%Vp(z) = Kp * E(z)

%Ci = Vi / E = Ki * Tm z / (z-1) = Ki * Tm /(1-z^-1)
%Vi(z) = Ki * Tm /(1-z^-1) * E(z)

%Cd = Vd / E = Kd *(1-z^-1)/Tm
%Vd(z) = Kd * (1-z^-1)/Tm * E(z)

%Vp[n] = Kp * e[n]
%Vi[n] = Ki * Tm * e[n] + vi[n-1]
%Vd[n] = Kd * Tm / e[n-1]
%% Controlador PIDstd
Kp = 3.833;
Ti = 0.6;
Td = 0.002;
Cpidstd = pidstd(Kp,Ti,Td,'Ts',Tm, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler')

%% Controlador PID

Ki = Kp/Ti;
Kd = Kp*Td;
Cpid = pid(Kp,Ki,Kd,'Ts',Tm, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler')


%% Simulacion con modelos
sim('modelo_motor_P4_2.slx')