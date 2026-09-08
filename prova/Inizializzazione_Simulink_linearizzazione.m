clear
clc
close all

%% Parametri modello

mm = 1000; % massa carico [kg]
mc = 300;  % massa carrello [kg]
cm = 100;  % coefficiente attrito carico [N/(m/s)]
cc = 5e3;  % coefficiente attrito carrello [N/(m/s)]
L = 2.5;   % Lunghezza cavo rigido [m]
g = 9.81;  % Accelerazione di gravit� [m/s^2]

% Tempo di simulazione
dt = 0.01;        % Tempo di campionamento [s]
tempo = 0:dt:30;  % Vettore tempo di simulazione [s]


%% Inizializzazione Stato all'equilibrio
alpha_0_eq = pi/2;   % Condizione iniziale alpha
alpha_dot_0_eq = 0;  % Condizione iniziale velocit� angolare
xc_0_eq = 0;         % Condizione iniziale posizione carrello
xc_dot_0_eq = 0;     % Condizione iniziale velocit� carrello

%% Lancio Simulazione per Linearizzazione
sim('modello_carroponte_linearizzazione.slx');

%% Estraggo le matrici (A, B, C, D) del sistema linearizzato

A = modello_carroponte_alpha_Timed_Based_Linearization.a;
B = modello_carroponte_alpha_Timed_Based_Linearization.b;
C = modello_carroponte_alpha_Timed_Based_Linearization.c;
D = modello_carroponte_alpha_Timed_Based_Linearization.d;

%% Funzione di trasferimento
sys = ss(A, B, C, D);
G = tf(sys); 
%In alternativa: s = tf('s'); --> G = C * inv(s*eye(4) * B + D;

% Bode
figure
bode(G);

% Transfer function input to angle
Ga = G(1);

% Transfer function input to position
Gxc = G(2);

%% Poles and zeros
figure;
subplot(2,1,1)
pzmap(Ga);
title('Poli Zeri Ga')
grid on;
hold on;
subplot(2,1,2)
pzmap(Gxc);
title('Poli Zeri Gxc')
grid on;

%sistema solo stabile, non asintoticamente ho polo in origine
%confronto tra sistema reale (non linearizzato) e lineare ottenuto da
%matrici ABCD, uso forzante esterna sinusoidale

%intorno a eq. ottimo confronto, ma continuando con perturbazione mi
%distacco ancora (ho comunque modellato con approssimazione). ne deriva un
%certo accoppiamento perché sono uscito da alcune condizioni di range di
%equilibrio, implementerç controllore per rimanere in range adeguato,
%altrimenti funziona alla ceca