clc 
clear all
close all

data = readmatrix('simulation_output.csv');  
% data = readmatrix('simulation_output_alternative.csv');  

t   = data(:,1);
RTx = data(:,2);
RTy = data(:,3);
RTz = data(:,4);
RIx = data(:,5);
RIy = data(:,6);
RIz = data(:,7);
VTx = data(:,8);
VTy = data(:,9);
VTz = data(:,10);
VIx = data(:,11);
VIy = data(:,12);
VIz = data(:,13);

% Posições e velocidades relativas (equivalente ao teu script)
RTIx = RTx - RIx;
RTIy = RTy - RIy;
RTIz = RTz - RIz;

VTIx = VTx - VIx;
VTIy = VTy - VIy;
VTIz = VTz - VIz;

% Distância (norma) e velocidade relativa (norma)
RTI = sqrt(RTIx.^2 + RTIy.^2 + RTIz.^2);
VTI = sqrt(VTIx.^2 + VTIy.^2 + VTIz.^2);

% Direção da linha de visada
R   = [RTIx RTIy RTIz];      % vetor relativo
LAMBDA = R ./ RTI;           % versor LOS

Rp  = [VTIx VTIy VTIz];      % velocidade relativa
Vh  = zeros(length(t),1);    % componente na LOS
for i = 1:length(t)
    Vh(i) = dot(Rp(i,:), LAMBDA(i,:));
end

% Índice da interceptação (igual ideia do seu script)
threshold = 1;  % em metros, por exemplo
miss_index = find(abs(RTI) <= threshold, 1);

if isempty(miss_index)
    % se nunca chegou tão perto assim, pega último ponto
    miss_index = length(t);
end

fprintf('Distância relativa no instante da interceptação (m): %.4f\n', RTI(miss_index));
fprintf('Tempo até a interceptação (s): %.6f\n', t(miss_index));

figure(1)
plot3(RTx(1:miss_index), RTy(1:miss_index), RTz(1:miss_index), ...
    'r--', 'LineWidth', 2); hold on;
plot3(RIx(1:miss_index), RIy(1:miss_index), RIz(1:miss_index), ...
    'b'  , 'LineWidth', 2); hold on;
plot3(RTx(1), RTy(1), RTz(1), 'or', 'linewidth', 2);
plot3(RIx(1), RIy(1), RIz(1), 'ob', 'linewidth', 2);
xlabel('Posição X [m]', 'fontsize', 16);
ylabel('Posição Y [m]', 'fontsize', 16);
zlabel('Posição Z [m]', 'fontsize', 16);
title('Pronav por ZEM PLUS');
set(gca, 'fontsize', 16);
set(gcf, 'color', 'w');
grid on

figure(2)
subplot(2,1,1);
plot(t(1:miss_index), RTI(1:miss_index), 'b', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 8);
ylabel('Distância Relativa [m]', 'FontSize', 8);
grid on;

subplot(2,1,2);
plot(t(1:miss_index), Vh(1:miss_index), 'b', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 8);
ylabel('Velocidade na LOS [m/s]', 'FontSize', 8);
grid on;


set(gcf, 'Color', 'w');
