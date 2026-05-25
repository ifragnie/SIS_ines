clc;
clear;
close all;

%% Charger CSV
T = readtable('fft_output.csv');

%% Vérification format
if ~ismember('FFT_ID', T.Properties.VariableNames)
    error('Colonne FFT_ID introuvable');
end

%% IDs FFT
fftID = T.FFT_ID;

%% Extraction des magnitudes (toutes les colonnes sauf FFT_ID)
magCols = ~strcmp(T.Properties.VariableNames,'FFT_ID');
magnitudeMatrix = table2array(T(:, magCols));

%% ================================
%  FILTRAGE FFT BINS
%  garder uniquement 7 → 255
%% ================================
magnitudeMatrix = magnitudeMatrix(:, 7:255);

[numFFT, fftSize] = size(magnitudeMatrix);

%% Axe fréquentiel
indexVals = 7:255;

%% Figure
fig = figure('Name','FFT Viewer', ...
             'Position',[100 100 900 500]);

ax = axes(fig, ...
          'Position',[0.08 0.2 0.88 0.72]);

%% FFT initiale
currentFFT = 1;

plotHandle = plot(ax, ...
    indexVals, ...
    magnitudeMatrix(currentFFT,:), ...
    'LineWidth', 1.5);

grid(ax, 'on');
xlabel(ax, 'Indice fréquentiel');
ylabel(ax, 'Magnitude');

title(ax, ['FFT #' num2str(fftID(currentFFT))]);

%% Texte affichage FFT ID
textHandle = uicontrol('Style','text', ...
    'Units','normalized', ...
    'Position',[0.42 0.11 0.2 0.04], ...
    'String',['FFT #' num2str(fftID(currentFFT))], ...
    'FontSize',12);

%% Slider
if numFFT > 1
    sliderStep = [1/(numFFT-1) 0.1];
else
    sliderStep = [1 1];
end

sliderHandle = uicontrol('Style','slider', ...
    'Min',1, ...
    'Max',numFFT, ...
    'Value',1, ...
    'SliderStep',sliderStep, ...
    'Units','normalized', ...
    'Position',[0.15 0.05 0.7 0.05]);

%% Sauvegarde des données
setappdata(fig,'fftID',fftID);
setappdata(fig,'magnitudeMatrix',magnitudeMatrix);
setappdata(fig,'indexVals',indexVals);
setappdata(fig,'plotHandle',plotHandle);
setappdata(fig,'textHandle',textHandle);

%% Callback slider
sliderHandle.Callback = @(src,~) updateFFT(round(src.Value), fig);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FONCTION DE MISE À JOUR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateFFT(k, fig)

    fftID = getappdata(fig,'fftID');
    magnitudeMatrix = getappdata(fig,'magnitudeMatrix');
    indexVals = getappdata(fig,'indexVals');

    plotHandle = getappdata(fig,'plotHandle');
    textHandle = getappdata(fig,'textHandle');
    sliderHandle = findobj(fig,'Style','slider');

    k = max(1, min(k, length(fftID)));

    %% Update plot
    set(plotHandle, ...
        'XData', indexVals, ...
        'YData', magnitudeMatrix(k,:));

    %% Update title
    ax = ancestor(plotHandle, 'axes');
    title(ax, ['FFT #' num2str(fftID(k))]);

    %% Update text
    set(textHandle, ...
        'String', ['FFT #' num2str(fftID(k))]);

    %% Sync slider
    sliderHandle.Value = k;
end