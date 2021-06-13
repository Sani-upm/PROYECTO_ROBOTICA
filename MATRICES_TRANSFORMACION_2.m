%% OBTINER LAS MATRICES DE TRANSFORMACIÓN A PARTIR DEL DH
%%clear all,clc


%% Definimos nuestras q
syms v1; 
syms v2; 
syms v3; 
syms v4;

%% Definimos las longitudes

syms L1;
syms L2;
syms L3;
syms offset;
syms offset_codo;
syms offset_gripper;


%% Con el DH calculamos las matrices de transformacion
%% 0 -> 1
A1 = trchain('Rz(pi/2)   Tz(v1)            Tx(0) Rx(0)')
%% 1 -> 2
A2 = trchain('Rz(-v2)    Tz(offset)        Tx(L1)Rx(0)')
%% 2 -> 3
A3 = trchain('Rz(-v3)    Tz(-offset_codo)   Tx(L2)Rx(pi)')
%% 3 -> 4
A4 = trchain('Rz(v4)     Tz(offset_gripper)Tx(0) Rx(0)')


%% Matriz de transformacion
A_sdr = A1 * A2 * A3 * A4

 