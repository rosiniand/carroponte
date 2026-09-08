clear; 
close all;
clc;

%% Caricamento in memoria dei parametri del sistema
% Parametri da definire: 
%  . mc (massa carrello) e mm (massa sospesa)
%  . cc e cm (costanti di smorzamento)
%  . L (lunghezza cavo)
%  . g (costante gravitazionale)
mm = 1000;      % kg
mc = 300;      % kg
cc = 5000;      % N*s/m
cm = 100;      % N*s/m
L = 2.5;       % m
g = 9.81;       % m/s^2
perc=8/9;
%% Definizione delle condizioni iniziali
% Definire condizioni iniziali per gli stati del sistema.
% Per esempio, si può prendere l'equilibrio stabile come condizione iniziale 
xc_0 = 0;               % m
xc_dot_0 = 0;           % m/s
alpha_0 = pi/2*perc;            % rad
alpha_dot_0 = 0;        % rad/s
%% Tempo di simulazione
dt=0.01;                %tempo di campionamento
tempo=0:dt:30;          %tempo di simulazione
