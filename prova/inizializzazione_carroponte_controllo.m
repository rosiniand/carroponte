clear; 
close all;
clc;

%% Caricamento in memoria dei parametri del sistema
% Parametri da definire: 
%  . mc (massa carrello) e mm (massa sospesa)
%  . cc e cm (costanti di smorzamento)
%  . L (lunghezza cavo)
%  . g (costante gravitazionale)
mm = 1000;  % kg
mc = 300;   % kg
cc = 5000;  % N*s/m
cm = 100;   % N*s/m
L = 2.5;    % m
g = 9.81;   % m/s^2

%% Definizione delle condizioni iniziali
% Definire condizioni iniziali per gli stati del sistema.
% Per esempio, si può prendere l'equilibrio stabile come condizione iniziale 
xc_0 = 0;               % m
xc_dot_0 = 0;           % m/s
alpha_0 = 80 * pi/180;  % rad
alpha_dot_0 = 0;        % rad/s

%% Definizione condizioni iniziali per linearizzazione
alpha_dot_eq = 0;
alpha_eq = pi/2;
xc_dot_eq = 0;
xc_eq = 0;

%% Lancio simulink per linearizzazione
sim('modello_linearizzazione.slx');

%% Estraggo le matrici (A, B, C, D) del sistema linearizzato
A = modello_carroponte_alpha_Timed_Based_Linearization.a;
B = modello_carroponte_alpha_Timed_Based_Linearization.b;
C = modello_carroponte_alpha_Timed_Based_Linearization.c;
D = modello_carroponte_alpha_Timed_Based_Linearization.d;

sys = ss(A, B, C, D);
G = tf(sys);

bode(sys);

%% Poli e zeri
Galpha = G(1, 1);
Gx = G(2, 1);

figure;
pzmap(Galpha)

figure;
pzmap(Gx);

%% Task 1: Progettare un controllore Proporzionale 

% Sul file simulink modello_carroponte_controllo, create un controllore P 
% Fate il tuning a mano
open("modello_carroponte_controllo.slx");

%% Task 2: Tuning del controllore con pidTuner 
%o a mano L=R*G, o a caso simulando con sistemi assolutamente stabili, se
%processi in cui rischio di rompere non posso
pidTuner(Gx);

%% Task 3: Progetto di un regolatore con Cancellazione poli-zeri

% Troviamo zeri e poli complessi coniugati della funzione di
% trasferimento
[z,p,k]= zpkdata(Gx, 'v');

% Proviamo a cancellare zeri e poli definendo una TF con poli e zeri
% dove vogliamo noi

% Hint: una soluzione comoda richiede l'uso dell'unità immaginaria i 
i = sqrt(-1);

Reg_ZPK = p(3)*p(4)/z(1)*z(2);

%% Task 4: Implementazione su simulink!

% Create un secondo anello di feedback e confrontate i risultati!
open("modello_carroponte_controllo.slx"); 
