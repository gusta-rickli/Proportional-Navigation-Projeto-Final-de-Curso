clc 
clear all
close all

%% Parâmetros de ProNav e Engajamento
aT = [1.2 -1 0.2]';
tf = 20;
dt = 1e-3;

% Ganho do Pronav por ZEM
Np = 3;
H = 650;  % Parâmetro de design

% Condições Iniciais de Engajamento (m)
RTx = 22;
RTy = 10;
RTz = 30;

RIx = 0;
RIy = 0;
RIz = 0;

% componentes da velocidade no sistema de coordenadas inercial (m/s)
VTx = 5;
VTy = 6;
VTz = 0;

VIx = 0;
VIy = 0;
VIz = 0;

% se as velocidades tivessem nos eixos do corpo, teria que converter

% Vetor de condição inicial
y0 = [RTx, RTy, RTz, RIx, RIy, RIz, VTx, VTy, VTz, VIx, VIy, VIz];

%% Simulação - integrar o engajamento 3D 

% --------------------------------------------------------------
% PRONAV POR ZEM COM TERMO ADICIONAL
% Linha de tempo discretizada
t = 0 : dt : tf-dt; nt = length(t);
% Pré-alocar matriz de solução para velocidade
yout_zem_p = zeros(nt,length(y0));

% Atribuir condição inicial
yout_zem_p(1,:) = y0;

% Integrar com passo de tempo constante, dt [s]
y_zem_p = y0;

% Pré-alocar para aceleração
aI_zem_p = zeros(nt, 3);

for j = 1 : nt-1
    [s1, aI] = pronav_ZEM_plus(y_zem_p, Np, H, aT);
    [s2, ~] = pronav_ZEM_plus(y_zem_p + dt*s1/2, Np, H, aT);
    [s3, ~] = pronav_ZEM_plus(y_zem_p + dt*s2/2, Np, H, aT);
    [s4, ~] = pronav_ZEM_plus(y_zem_p + dt*s3, Np, H, aT);
    
    y_zem_p = y_zem_p + (dt/6)*(s1 + 2*s2 + 2*s3 + s4);
    yout_zem_p(j+1,:) = y_zem_p;

    aI_zem_p(j,:) = aI'; % Armazena aceleração
end

y_zem_p = yout_zem_p;

%% Pós-processamento dos resultados da simulação
% --------------------------------------------------------------
% Ponteiros para os estados
sel_RTx = 1;
sel_RTy = 2;
sel_RTz = 3;
sel_RIx = 4;
sel_RIy = 5;
sel_RIz = 6;
sel_VTx = 7;
sel_VTy = 8;
sel_VTz = 9;
sel_VIx = 10;
sel_VIy = 11;
sel_VIz = 12;

%% Visualizar Resultados

% --------------------------------------------------------------
% posições e velocidades relativas pelo ZEM plus
RTIx_zem_p = y_zem_p(:,sel_RTx) - y_zem_p(:,sel_RIx);
RTIy_zem_p = y_zem_p(:,sel_RTy) - y_zem_p(:,sel_RIy);
RTIz_zem_p = y_zem_p(:,sel_RTz) - y_zem_p(:,sel_RIz);

VTIx_zem_p = y_zem_p(:,sel_VTx) - y_zem_p(:,sel_VIx);
VTIy_zem_p = y_zem_p(:,sel_VTy) - y_zem_p(:,sel_VIy);
VTIz_zem_p = y_zem_p(:,sel_VTz) - y_zem_p(:,sel_VIz);

% distância (norma)
RTI_zem_p = sqrt(RTIx_zem_p.^2 + RTIy_zem_p.^2 + RTIz_zem_p.^2);

% velocidade (norma)
VTI_zem_p = sqrt(VTIx_zem_p.^2 + VTIy_zem_p.^2 + VTIz_zem_p.^2);

% aceleração (norma)
AI_zem_p = zeros(nt,1);
for i = 1:nt
    AI_zem_p(i) = sqrt(aI_zem_p(i, 1)^2 + aI_zem_p(i, 2)^2 + aI_zem_p(i, 3)^2);
end

% Direção da Linha de Visão
R_zem_p = [RTIx_zem_p RTIy_zem_p RTIz_zem_p]; % vetor da distância relativa
LAMBDA_zem_p = R_zem_p./RTI_zem_p;          % versor que representa direção da LOS

Rp_zem_p = [VTIx_zem_p VTIy_zem_p VTIz_zem_p]; % vetor da velocidade relativa

% vetor da velocidade relativa na LOS
Vh_zem_p = zeros(nt, 1); % Inicializa o vetor de resultados
for i = 1:nt
    Vh_zem_p(i) = dot(Rp_zem_p(i, :), LAMBDA_zem_p(i, :)); % Produto escalar linha a linha
end

% --------------------------------------------------------------
% Número de passos de tempo por passo de tempo de plotagem
dt_index = .1/dt;

% Número de passos de tempo bruto
nt = length(t);

% Índice de tempo (tempo até a interceptação)
% miss_index_zem = find(min(abs(RTI_zem)) == abs(RTI_zem));

threshold = 1; % Define o valor limite
miss_index_zem_p = find(abs(RTI_zem_p) <= threshold, 1);

fprintf('Distância relativa no instante da interceptação (m): %.4f\n', RTI_zem_p(miss_index_zem_p));
fprintf('Tempo até a interceptação (s): %.6f\n', t(miss_index_zem_p));

% Gráficos básicos de engajamento
% --------------------------------------------------------------

figure(3)
plot3(y_zem_p(1:miss_index_zem_p,sel_RTx),y_zem_p(1:miss_index_zem_p,sel_RTy), y_zem_p(1:miss_index_zem_p,sel_RTz), ...
    'r--', 'linewidth', 2); hold on
plot3(y_zem_p(1:miss_index_zem_p,sel_RIx),y_zem_p(1:miss_index_zem_p,sel_RIy), y_zem_p(1:miss_index_zem_p,sel_RIz), ...
    'b','linewidth', 2); hold on
plot3(y_zem_p(1,sel_RIx),y_zem_p(1,sel_RIy), y_zem_p(1,sel_RIz), 'ob', 'linewidth', 2);
plot3(y_zem_p(1,sel_RTx),y_zem_p(1,sel_RTy), y_zem_p(1,sel_RTz), 'or', 'linewidth', 2);
xlabel('Posição X [m]', 'fontsize', 16);
ylabel('Posição Y [m]', 'fontsize', 16);
zlabel('Posição Z [m]', 'fontsize', 16);
title('Pronav por ZEM PLUS');
set(gca, 'fontsize', 16);
set(gcf, 'color', 'w');
grid on

figure(4)
subplot(3,1,1); % Subplot para RTI_zem
plot(t(1:miss_index_zem_p), RTI_zem_p(1:miss_index_zem_p), 'b', 'LineWidth', 2);
xlabel('Tempo [s]', 'fontsize', 8);
ylabel('Distância Relativa [m]', 'fontsize', 8);
title('Pronav por ZEM PLUS', 'fontsize', 16);
grid on;

subplot(3,1,2); % Subplot para Vh_zem
plot(t(1:miss_index_zem_p), Vh_zem_p(1:miss_index_zem_p), 'b', 'LineWidth', 2);
xlabel('Tempo [s]', 'fontsize', 8);
ylabel('Velocidade na LOS [m/s]', 'fontsize', 8);
grid on;

subplot(3,1,3); % Subplot para AI_fv
plot(t(1:miss_index_zem_p), AI_zem_p(1:miss_index_zem_p), 'b', 'LineWidth', 2);
xlabel('Tempo [s]', 'fontsize', 8);
ylabel('Aceleração do Intercep. [m/s^2]', 'fontsize', 8);
grid on;

set(gcf, 'color', 'w'); % Define o fundo da figura como branco

%% Função que calcula o vetor de estado
% --------------------------------------------------------------

% --------------------------------------------------------------
function [dy, aI] = pronav_ZEM_plus(y, Np, H, aT)

% Ponteiros para os estados
sel_RT1 = 1;
sel_RT2 = 2;
sel_RT3 = 3;
sel_RI1 = 4;
sel_RI2 = 5;
sel_RI3 = 6;
sel_VT1 = 7;
sel_VT2 = 8;
sel_VT3 = 9;
sel_VI1 = 10;
sel_VI2 = 11;
sel_VI3 = 12;

% Pré-alocar vetor do lado esquerdo
dy = [0 0 0 0 0 0 0 0 0 0 0 0];

% posições e velocidades relativas
RTI1 = y(sel_RT1) - y(sel_RI1);
RTI2 = y(sel_RT2) - y(sel_RI2);
RTI3 = y(sel_RT3) - y(sel_RI3);

VTI1 = y(sel_VT1) - y(sel_VI1);
VTI2 = y(sel_VT2) - y(sel_VI2);
VTI3 = y(sel_VT3) - y(sel_VI3);

% Direção da Linha de Visão
R = [RTI1 RTI2 RTI3]'; % vetor da distância relativa (z1)
r = norm(R);           % módulo da distânicia relativa
LAMBDA = R/r;          % versor que representa direção da LOS
V = [VTI1 VTI2 VTI3]';
v = norm(V) ;

tgo = abs(r/v);

ZEM = (R + V*tgo) ;

T = (1/H)*(r^2)*(LAMBDA);

aI = Np*ZEM/tgo^2 + T;

% Cálculos do lado direito das equações diferenciais y = [RTx, RTy, RTz, RIx, RIy, RIz, VTx, VTy, VTz, VIx, VIy, VIz]
dy(1) = y(sel_VT1);
dy(2) = y(sel_VT2);
dy(3) = y(sel_VT3);
dy(4) = y(sel_VI1);
dy(5) = y(sel_VI2);
dy(6) = y(sel_VI3);
dy(7) = aT(1);
dy(8) = aT(2);
dy(9) = aT(3);

% Aceleração do interceptador
dy(10) = aI(1); 
dy(11) = aI(2); 
dy(12) = aI(3); 

end
