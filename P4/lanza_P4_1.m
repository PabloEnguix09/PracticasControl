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

%% Sección: Modelo lineal del motor DC

% Variable de Laplace
s = tf([1 0],1);

% Polos y ganancia
a = 1.77; % Polo dominante del motor
b = a*80; % 2º polo del motor (no dominante)
k = 3.6; % Ganancia del motor 

% Funcion de transferencia en dominio continuo
G = k*a/(s+a)*b/(s+b);

% Funcion de transferencia en dominio discreto
Gz = c2d(G,Tm);


%% Control P
Kp1 = 3.833;

%% Control PI
Kp2 = 3.833;
Ti2 = 0.6;

%% Control PID
Kp3 = 3.833;
Ti3 = 0.6;
Td3 = 0.002;


%% Simulacion con modelos
sim('modelo_motor_P4_1.slx')

%% Grafica respuesta
figure (101)
plot(t,v_resp(1:Lt,1),t,v_resp(1:Lt,2),t,v_resp(1:Lt,3),t,v_resp(1:Lt,4))
grid
legend('\omega_{ref}','\omega Control P','\omega Control PI','\omega Control PID','Location','southeast')
xlabel('t (s)')
ylabel('Amplitud')