clear all; close all; clc;

% 
% ES827 - Robotica Industrial - 1s2022
% 
% Grupo 3 - Robo ABB IRB 120
% 
% Arnaud Bosquillon de Jarcy   203079
% Gabriel de Freitas Leite     216180
% Igor Barros Teixeira         217947
% Matheus Santos Sano          222370
% 

% Definicao das massas dos links
m1=3.060;
m2=3.908;
m3=2.940;
m4=1.320;
m5=0.546;
m6=0.0136;
m_efetuador=3;

% Criacao dos elos com dimensoes e massas
L1 = Link('a',0,'alpha',-pi/2,'d',0.29,'m',m1);
L2 = Link('a',0.27,'alpha',0,'d',0,'m',m2);
L3 = Link('a',0.07,'alpha',-pi/2,'d',0,'m',m3);
L4 = Link('a',0,'alpha',pi/2,'d',0.302,'m',m4);
L5 = Link('a',0,'alpha',-pi/2,'d',0,'m',m5);
L6 = Link('a',0,'alpha',0,'d',0.072,'m',m6+m_efetuador);

% Centro de massa no centro de cada elo
r1=0.29/2;
r2=0.27/2;
r3=0.134/2;
r4=(0.302-0.134)/2;
r5=0/2;
r6=0.072/2;

L1.r = [0 0 r1];
L2.r = [0 0 r2];
L3.r = [r3 0 0];
L4.r = [r4 0 0];
L5.r = [0 0 0]; % Link 5 é embutido com o 6
L6.r = [r6 0 0];

% Considerando momento de inercia em torno do eixo igual a zero (delgado)
L1.I = [m1*r1^2 m1*r1^2 0];
L2.I = [m2*r2^2 m2*r2^2 0];
L3.I = [m3*r3^2 0 m3*r3^2];
L4.I = [m4*r4^2 0 m4*r4^2];
L5.I = [0 0 0];
L6.I = [m6*r6^2+m_efetuador*(2*r6)^2 0 m6*r6^2+m_efetuador*(2*r6)^2];

bot = SerialLink([L1 L2 L3 L4 L5 L6],'name','IRB 120','gravity',[0 0 9.81])

q=[0 -pi/2 0 0 0 0];

% Cinematica direta
T=bot.fkine(q);

% Cinematica inversa
qi=bot.ikine(T,[1 1 1 0 0 0]);

% Montagem da elipse 1
x1=0.464;
y1=0.195;
x2=0.1;
y2=0.3;
e=0.7;
a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
b = a*sqrt(1-e^2);
t = linspace(0,2*pi,100);
X = a*cos(t);
Y = b*sin(t);
w = atan2(y2-y1,x2-x1);
x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
tamanho=size(x);
z=zeros(1,tamanho(2));

figure(1)
plot(x,y,'red')
title('Calculando trajetórias...')
hold on
xlim([-1,1])
ylim([-1,1])
zlim([0,1])

% Calculos de trajetorias e animacao do movimento
p=[x;y;z];
T=[];

for i = 1:length(t)
   aux= transl(p(1:3,i));
   T = cat(2,T,aux);
end

Q=[];
Q = cat(2,Q,q');

for i = 1:length(t)
    aux1=[T(1:4,4*i-3) T(1:4,4*i-2) T(1:4,4*i-1) T(1:4,4*i)];
    aux2=bot.ikine(aux1, 'mask', [1 1 1 0 0 0]);
    Q = cat(2,Q,aux2');
end

Q = cat(2,Q,q');
t1=linspace(0,2,20);

aux=jtraj(Q(1:6,1)',Q(1:6,2)',t1);

Qtotal=[];
Qtotal = cat(2,Qtotal,aux');

for i = 2:length(t)+1
    Qtotal = cat(2,Qtotal,Q(1:6,i));
end

aux=jtraj(Q(1:6,length(t)+1)',Q(1:6,1)',t1);
Qtotal = cat(2,Qtotal,aux');

% Montagem da elipse 2
z1=0.6;
x1=0.1;
z2=0.2;
x2=0.2;
e=0.7;
a = 1/2*sqrt((z2-z1)^2+(x2-x1)^2);
b = a*sqrt(1-e^2);
t = linspace(0,2*pi,100);
Z = a*cos(t);
X = b*sin(t);
w = atan2(x2-x1,z2-z1);
z = (z1+z2)/2 + Z*cos(w) - X*sin(w);
x = (z1+z2)/2 + Z*sin(w) + X*cos(w);
tamanho=size(z);
y=-0.3.*ones(1,tamanho(2));
figure(1)
plot3(x,y,z,'red')

% Movimento
p=[x;y;z];
T=[];

for i = 1:length(t)
   aux= transl(p(1:3,i));
   T = cat(2,T,aux);
end

Q=[];
Q = cat(2,Q,q');

for i = 1:length(t)
    aux1=[T(1:4,4*i-3) T(1:4,4*i-2) T(1:4,4*i-1) T(1:4,4*i)];
    aux2=bot.ikine(aux1, 'mask', [1 1 1 0 0 0]);
    Q = cat(2,Q,aux2');
end

Q = cat(2,Q,q');
t1=linspace(0,2,20);
aux=jtraj(Q(1:6,1)',Q(1:6,2)',t1);
Qtotal = cat(2,Qtotal,aux');

for i = 2:length(t)+1
    Qtotal = cat(2,Qtotal,Q(1:6,i));
end

aux=jtraj(Q(1:6,length(t)+1)',Q(1:6,1)',t1);
Qtotal = cat(2,Qtotal,aux');

figure(1)
title('Mostrando animação de trajetória...')
bot.plot(Qtotal');
title('Movimento finalizado!')

%% Calculo de torque em cada junta

% Criacao dos vetores
J = [];
Qfinal = [];
QDfinal = [];
QDDfinal = [];

% Definicao de velocidade linear e rotacional
dX = [0.001 0.001 0.001 0 0 0]';

% Calculando as rotas novamente a partir de um novo q inicial (mesmos
% calculos deitos anteriormente para gerar a trajetoria das elipses
q=[0 -pi/6 -pi/6 0 pi/3 0];
% Cinematica direta
T=bot.fkine(q);
% Cinematica inversa
qi=bot.ikine(T,[1 1 1 0 0 0]);
% Montagem da elipse 1
x1=0.464;
y1=0.195;
x2=0.1;
y2=0.3;
e=0.7;
a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
b = a*sqrt(1-e^2);
t = linspace(0,2*pi,100);
X = a*cos(t);
Y = b*sin(t);
w = atan2(y2-y1,x2-x1);
x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
tamanho=size(x);
z=zeros(1,tamanho(2));
figure(1)
plot(x,y,'green')
title('Recalculando trajetórias...')
hold on
xlim([-1,1])
ylim([-1,1])
zlim([0,1])
% Calculos de trajetorias e animacao do movimento
p=[x;y;z];
T=[];
for i = 1:length(t)
   aux= transl(p(1:3,i));
   T = cat(2,T,aux);
end
Q=[];
Q = cat(2,Q,q');
for i = 1:length(t)
    aux1=[T(1:4,4*i-3) T(1:4,4*i-2) T(1:4,4*i-1) T(1:4,4*i)];
    aux2=bot.ikine(aux1, 'mask', [1 1 1 0 0 0]);
    Q = cat(2,Q,aux2');
end
Q = cat(2,Q,q');
t1=linspace(0,2,20);
aux=jtraj(Q(1:6,1)',Q(1:6,2)',t1);
Qtotal=[];
Qtotal = cat(2,Qtotal,aux');
for i = 2:length(t)+1
    Qtotal = cat(2,Qtotal,Q(1:6,i));
end
aux=jtraj(Q(1:6,length(t)+1)',Q(1:6,1)',t1);
Qtotal = cat(2,Qtotal,aux');
% Montagem da elipse 2
z1=0.6;
x1=0.1;
z2=0.2;
x2=0.2;
e=0.7;
a = 1/2*sqrt((z2-z1)^2+(x2-x1)^2);
b = a*sqrt(1-e^2);
t = linspace(0,2*pi,100);
Z = a*cos(t);
X = b*sin(t);
w = atan2(x2-x1,z2-z1);
z = (z1+z2)/2 + Z*cos(w) - X*sin(w);
x = (z1+z2)/2 + Z*sin(w) + X*cos(w);
tamanho=size(z);
y=-0.3.*ones(1,tamanho(2));
figure(1)
plot3(x,y,z,'green')
% Movimento
p=[x;y;z];
T=[];
for i = 1:length(t)
   aux= transl(p(1:3,i));
   T = cat(2,T,aux);
end
Q=[];
Q = cat(2,Q,q');
for i = 1:length(t)
    aux1=[T(1:4,4*i-3) T(1:4,4*i-2) T(1:4,4*i-1) T(1:4,4*i)];
    aux2=bot.ikine(aux1, 'mask', [1 1 1 0 0 0]);
    Q = cat(2,Q,aux2');
end
Q = cat(2,Q,q');
t1=linspace(0,2,20);
aux=jtraj(Q(1:6,1)',Q(1:6,2)',t1);
Qtotal = cat(2,Qtotal,aux');
for i = 2:length(t)+1 
    Qtotal = cat(2,Qtotal,Q(1:6,i));
end
aux=jtraj(Q(1:6,length(t)+1)',Q(1:6,1)',t1);
Qtotal = cat(2,Qtotal,aux');


% Calculos de pos, vel e acel nas trajetorias
for i= 1:length(Qtotal)-2

    Qatual = Qtotal(:,i);
    Qproximo = Qtotal(:,i+1);

    Jatual = jacob0(bot,Qatual');
    Jproximo = jacob0(bot,Qproximo');

    QDatual = Jatual\dX;
    QDproximo = Jproximo\dX;
    
    [Q_1,QD_1,QDD_1] = jtraj(Qtotal(:,i),Qtotal(:,i+1),2,QDatual,QDproximo);

    Qfinal = cat(1,Qfinal,Q_1);
    QDfinal = cat(1,QDfinal,QD_1);
    QDDfinal = cat(1,QDDfinal,QDD_1);

end

figure(1)
title('Roposicionando robô e realizando movimento após cálculos de torque...')
bot.plot(Qfinal);
title('Movimento finalizado!')

% Calculo dos torques
torque = bot.rne(Qfinal,QDfinal,QDDfinal);

% Plot do torque de cada motor
figure(2)
tamanho = length(torque);
t=0:tamanho-1;
plot(t,torque(:,1),t,torque(:,2),t,torque(:,3),t,torque(:,4),t,torque(:,5),t,torque(:,6),'LineWidth',1)
legend('motor 1','motor 2','motor 3','motor 4','motor 5','motor 6')
title('Torques nos elos do robô')
grid on

