clc;
clear;


%% Electrical Models
Vdc=600;
Lamf=0.1;           %PM flux linkage, Lamdqsr are stator flux linkages
Rs=33.41/2;          %Stator Resistance
Lds=(31.07e-3)/2;    %Stator inductance
Lqs=(31.07e-3)/2;
Is_Rated=1.77;


%% Mechanical Models
Jm=7.5e-6;        %Rotor Inertia  
Bm=0;              %Rotor Damping
Te_Rated=1.1;
Thetarm_Init=0;
Wrm_Init=0;
Tmax=1.4;
p=3;                %Pole pairs


%% Current controller gain setting

fsw=5e3;        % Switching frequency
Tsw=1/fsw;      % Switching period

fs=2*fsw;       % Samping frequency (double sampling)
Ts=1/fs;        % Samping period

CC.Wc=fs*2*pi/20;               % Current controller BW
CC.Is_lim=Is_Rated;             % Current constraint
CC.Ra=0;%10*Rs;                    % Active damping
CC.Kp_Mat=CC.Wc*[Lds 0;0 Lqs];
CC.Ki_Mat=CC.Wc*(Rs+CC.Ra)*eye(2)*1;
CC.Ka_Mat=inv(CC.Kp_Mat);





%% Speed Controller Gain setting
SC.Wc=CC.Wc/10;           % Speed controller BW
SC.Te_lim=Tmax;           % Torque limit for Speed controller
SC.Kp=Jm*SC.Wc;
SC.Ki=SC.Kp*SC.Wc/5;
SC.Ka=2/SC.Kp;


%% Simulation Parameters
Stop_Time=1.5;
Step_Time=[0.3 0.6 0.9 1.2];

% Te_Ref_Set=[0.4 0.6 0.3 0 0.8]*Te_Rated;
Te_Ref_Set = [0.5 0.5 0.5 0.5 0.5]*Te_Rated;
Wrpm_Ref_Set=[4000 5000 3000 6000 4000];

% Wrpm_Ref_Set=[4000 4000 4000 4000 4000];
Te_SlewRate=inf;
% TL=[0.5 0.5 0.5 0.8 0.8]*Te_Rated;
TL=Te_Ref_Set;
Mode.Ctrl=2;

