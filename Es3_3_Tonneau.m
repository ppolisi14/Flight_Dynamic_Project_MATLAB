%% Esercizio 3.3 Manovra di tonneau
clc; close all; clearvars;
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
%% DATI
t_fin=130;
u0=convvel(380,'km/h','m/s');

%Quota e posizione iniziale
h0 = 100; %quota iniziale in m
x0 = 0;
y0 = 0;
z0 = -h0;

%Definizione dei valori max e dei punti su cui poi applicare
%interpolante cubica.
q_max = convangvel(2,'deg/s','rad/s');
p_max = convangvel(4,'deg/s','rad/s');  

tq_points = (0:0.075:1).*t_fin; %14 istanti t
q_points = [0, 0.2*q_max, 0.5*q_max, 0.9*q_max,...
    q_max, q_max, q_max, q_max, q_max, 0.9*q_max,...
    0.72*q_max, 0*q_max, 0*q_max, 0];
     %si è costruita con un processo trial and error

tp_points = (0:0.075:1).*t_fin; %14 istanti t
p_points = [0, 0.2*p_max, 0.5*p_max,...
    0.9*p_max, p_max, p_max, p_max,...
    p_max, p_max, p_max, 0.65*p_max, 0*p_max, 0*p_max, 0];

tu_points = [0, 0.1, 0.2, 0.3, 0.4,...
    0.5, 0.6, 0.7, 0.8, 1]*t_fin;
u_points = [u0, 0.7*u0, 0.9*u0, 0.95*u0, 0.98*u0, u0, u0,...
    u0, u0, u0]; 


q = @(t) interp1(tq_points,q_points,t,'pchip');
p = @(t) interp1(tp_points,p_points,t,'pchip'); 
r = @(t) t.*0; 

u = @(t) interp1(tu_points, u_points, t, 'pchip');
v = @(t) t.*0; 
w = @(t) t.*0;
%per semplicità si suppone che v,w siano identicamente nulle


%Vettore dei tempi
v_time=linspace(0,t_fin,200);

%Plot delle storie temporali
figure;
plot(v_time,convangvel(p(v_time),'rad/s','deg/s'),...
    v_time,convangvel(q(v_time),'rad/s','deg/s'),...
    v_time,r(v_time),'LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('($^{\circ}$/s)','rotation',0,...
    "Position",[-10,3.1,-2]);
%text(1.6,50,'q(t)'); text(3,5,'p(t)'); text(6,5,'r(t)');
legend('p(t)','q(t)','r(t)'); axis([0 t_fin 0 5])
% print('vangton','-djpeg','-r1200');

figure;
plot(v_time,u(v_time),v_time,v(v_time),...
    v_time,w(v_time),'LineWidth',1.5);
grid on; xlabel('t (s)'); ylabel('(m/s)','rotation',0);
text(20,105,'u(t)'); text(20,8,'v(t)');
text(80,8,'w(t)');
%legend('u(t)','v(t)','w(t)'); 
axis([0 t_fin -40 200]);
% print('vton','-djpeg','-r1200');

%% RISOLUZIONE
%Si risolvono le equazioni del quaternione con ode45

%Angoli di Eulero iniziali: volo ad ali livellate,
%fus. orizzontale e prua in direzione Nord
psi0=0; theta0=0; phi0=0;

%Si passa al quaternione
Q_0=angle2quat(psi0,theta0,phi0);  %di default 321

%Funzione per l'ODE45 (RHS 3.67: Qpunto=[]*Q_0)
dQuatdt= @(t,Q) ...
    1/2 * [0,  -p(t), -q(t), -r(t);
          p(t), 0,     r(t), -q(t);
          q(t),-r(t),  0   ,  p(t);
          r(t), q(t), -p(t),  0     ] * Q;

options = odeset( 'RelTol',1e-9,...
    'AbsTol',1e-9*ones(1,4));

[Time, Quat] = ode45(dQuatdt, v_time ,Q_0,options);

% Quaternione
q0=Quat(:,1);
q1=Quat(:,2);
q2=Quat(:,3);
q3=Quat(:,4);

figure(3)
plot(v_time,q0,v_time,q1,v_time,q2,v_time,q3,"LineWidth",1.5)
legend('$q_{0(t)}$','$q_{x(t)}$','$q_{y(t)}$',...
    '$q_{z(t)}$','Location', 'best') 
xlabel('t (s)'); ylabel('q','rotation',0);grid on;
ylim([-1.1 1.1]); xlim([0 t_fin]);
% print('quatton','-djpeg','-r1200');
%Si ricavano gli angoli di Eulero
[psi,theta,phi] = quat2angle([Quat]);

figure(4)
plot( v_time, convang(phi,'rad','deg'),'.-b'); 
hold on;
plot( v_time, convang(theta,'rad','deg'),'-' )
plot( v_time, convang(psi,'rad','deg'), 'k');
xlabel('t (s)'); ylabel('($^{\circ}$)'); 
legend('$\phi$','$\theta$','$\psi$',...
    'Location', 'NorthEast') 
grid on; axis([0 t_fin -200 200]);
% print('euleroton','-djpeg','-r1200');

% Si calcolano le componenti della posizione
% del baricentro in assi Terra con ODE45 (eq. 3.28)

% Si crea la matrice quat con quaternioni interpolati
   Quatinterp = @(t) ...
[interp1(v_time,Quat(:,1),t), ...
interp1(v_time,Quat(:,2),t), ...
interp1(v_time,Quat(:,3),t), ...
interp1(v_time,Quat(:,4),t)];

dPositionEdt = @(t,Position) ...
transpose(quat2dcm(Quatinterp(t)))*...
[u(t);v(t);w(t)]; %RHS 3.28
% quat2dcm è una funzione che restituisce
% la matrice della 3.25 E to B,
% mentre si necessita di quella B to E 
%(3.28) che è l'inversa (trasposta)

options = odeset('RelTol', 1e-3,...
    'AbsTol', 1e-3*ones(3,1));  
 
Position = [x0;y0;z0];   

[Time, vPositionE] = ode45(dPositionEdt, v_time,...
    Position, options);                     
    
xE  = vPositionE(:,1);    
yE  = vPositionE(:,2);    
zE  = vPositionE(:,3);

figure(5)
plot(v_time, xE,"LineWidth",1.5); hold on;
plot(v_time, yE,"LineWidth",1.5) 
plot(v_time, zE,"LineWidth",1.5)
grid on;
axis([0 t_fin -4000 13000]);
legend('$x_{E,G}$','$y_{E,G}$','$z_{E,G}$','Location', 'Best')   
xlabel('t (s)'); ylabel('(m)');
% print('positionton','-djpeg','-r1200');

%% Rappresentazione della traiettoria 3D

% Impostazione della rappresentazione 3D della 
%traiettoria. daspect definisce i rapporti di 
%proporzionalità tra gli assi nella visualizzazione.
Figure_1=figure(6);
grid on; hold on;
light('Position', [1 0 -4], 'Style', 'local');  
set(gca,'XDir','reverse');  
set(gca,'ZDir','reverse');
daspect([1 1 1]);

% Si carica il modello ingrandito di 600 volte.
ScaleFactor = 600.0;
shape = loadAircraftMAT('aircraft_mig29',ScaleFactor);
Position_E_G = [xE, yE, zE];
EulerAngles_BE = [psi, theta, phi];

% Impostazione assi. Options.samples %
%definisce il numero di eventi da rappresentare
options.samples = [1:15:length(v_time)];
options.theView = [44 16];
options.bodyAxes.show = true;   
options.bodyAxes.lineWidth = 2.5;
options.bodyAxes.magX = 1.5*ScaleFactor ;
options.bodyAxes.magY = 2.0*ScaleFactor ;
options.bodyAxes.magZ = 2.0*ScaleFactor ;

% Impostazione delle linee di traiettoria e ausiliarie.
options.helperLines.show = true;
options.helperLines.lineStyle = ':';
options.helperLines.lineColor = 'k';
options.helperLines.lineWidth = 1.5;
     
options.trajectory.show = true;
options.trajectory.lineStyle = '-';
options.trajectory.lineColor = 'k';
options.trajectory.lineWidth = 1.5;
 
% Rappresentazione della traiettoria e degli assi Terra.
plotTrajectoryAndBodyE(Figure_1, shape,...
    Position_E_G, EulerAngles_BE, options); 
%funzione definita dal professore
     
hold on;
Xmax = max([max(abs(Position_E_G(:,1))), 5]);
Ymax = max([max(abs(Position_E_G(:,2))), 5]);
Zmax = 0.05 * Xmax;
Position_E_0 = [0 0 0];
Maxima = [Xmax, Ymax, Zmax];
plotEarthAxes(Figure_1, Position_E_0, Maxima);
xlabel('$X_{E}$ (m)'); ylabel('$Y_{E}$ (m)');
zlabel('$Z_{E}$ (m)'); hold off;
% print('trajectoryton2','-djpeg','-r1200');

