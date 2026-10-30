clear;
close all;

rootDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(rootDir, 'data');
numPoints = 14;
numAntennas = 6;
windowSize = 35;

groundTruth = [
     7.46, -3.20;
     7.46,  0.00;
     7.46,  3.20;
     3.73, -3.20;
     3.73,  0.00;
     3.73,  3.20;
     0.00, -3.20;
     0.00,  3.20;
    -3.73, -3.20;
    -3.73,  0.00;
    -3.73,  3.20;
    -7.46, -3.20;
    -7.46,  0.00;
    -7.46,  3.20
];

data = cell(numPoints, 1);
for pointIndex = 1:numPoints
    filePath = fullfile(dataDir, sprintf('measurement_point_%02d.mat', pointIndex));
    loaded = load(filePath, 'data_log');
    samples = loaded.data_log;
    data{pointIndex} = samples;
end

firstPath = estimatePositions(data, 4, numAntennas, windowSize);
totalCir = estimatePositions(data, 5, numAntennas, windowSize);

set(groot, 'defaultAxesFontSize', 16);
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

figure(1);
layout = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
plotLocalization(nexttile(layout), totalCir, groundTruth, '(a) Total CIR power');
plotLocalization(nexttile(layout), firstPath, groundTruth, '(b) First-path power');

function positions = estimatePositions(data, powerColumn, numAntennas, windowSize)
    unitVectors = exp(1i * deg2rad((0:60:300)'));
    positions = cell(size(data));

    for pointIndex = 1:numel(data)
        samples = data{pointIndex};
        ranges = mean(reshape(samples(:, 3), numAntennas, []).', 2);
        power = reshape(samples(:, powerColumn), numAntennas, []).';
        power = movmean(power, windowSize, 1);
        direction = power * unitVectors;
        direction(abs(direction) == 0) = NaN;
        positions{pointIndex} = ranges .* direction ./ abs(direction);
    end
end

function plotLocalization(ax, positions, groundTruth, panelTitle)
    hold(ax, 'on');
    colors = lines(numel(positions));

    for pointIndex = 1:numel(positions)
        estimate = positions{pointIndex};
        scatter(ax, real(estimate), imag(estimate), 30, colors(pointIndex, :), 'x');
        plot(ax, groundTruth(pointIndex, 1), groundTruth(pointIndex, 2), 'ko', ...
            'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'k');
    end

    rectangle(ax, 'Position', [-11.4, -6.4, 22.8, 12.8], 'EdgeColor', 'k');
    title(ax, panelTitle);
    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    axis(ax, 'equal');
    xlim(ax, [-12, 12]);
    ylim(ax, [-7, 7]);
    grid(ax, 'on');
    box(ax, 'on');
end

