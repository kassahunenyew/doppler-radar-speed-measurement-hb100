% =========================================================================
% CLEAN DOPPLER FFT RADAR MATLAB SCRIPT (TOOLBOX-FREE)
% Beautified plotting + working slow motion detection
% =========================================================================

clear all; close all; clc;

%% USER SETTINGS
portName = "COM4";      % your COM port
baudRate = 115200;
FFT_SIZE = 512;
Fs = 10000;
maxTime = 20;           % record duration

%% OPEN SERIAL
s = serialport(portName, baudRate);
flush(s);
disp("Waiting for incoming radar data...");

%% STORAGE
tVec   = [];
fVec   = [];
vVec   = [];
rmsVec = [];

%% FIGURE SETUP (BEAUTIFIED)
freqAxis = (0:FFT_SIZE/2 - 1) * (Fs/FFT_SIZE);

figure('Color','w','Position',[100 80 1300 820]);
tiledlayout(4,1,"TileSpacing","compact","Padding","compact");

%% TIME DOMAIN
ax1   = nexttile;
hTime = plot(NaN,NaN,'LineWidth',1.5,'Color',[0 0.45 0.75]);
grid on;
title("Time Domain","FontSize",14,'FontWeight','bold');
ylabel("Amplitude","FontWeight","bold");
xlabel("Sample Index","FontWeight","bold");

%% FFT SPECTRUM
ax2  = nexttile;
hFFT = plot(NaN,NaN,'b','LineWidth',1.7);
hold on;
% Beautified peak marker
hPeak = plot(NaN,NaN,'ro', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','r', ...
    'LineWidth',2);
grid on;
title("FFT Spectrum","FontSize",14,'FontWeight','bold');
ylabel("Magnitude","FontWeight","bold");
xlabel("Frequency (Hz)","FontWeight","bold");

%% DOPPLER FREQUENCY vs TIME
ax3   = nexttile;
hFreq = plot(NaN,NaN,'r','LineWidth',2);
grid on;
title("Doppler Frequency vs Time","FontSize",14,'FontWeight','bold');
ylabel("Frequency (Hz)","FontWeight","bold");
xlabel("Time (s)","FontWeight","bold");

%% SPEED vs TIME
ax4    = nexttile;
hSpeed = plot(NaN,NaN,'g','LineWidth',2);
grid on;
title("Speed vs Time (km/h)","FontSize",14,'FontWeight','bold');
ylabel("Speed (km/h)","FontWeight","bold");
xlabel("Time (s)","FontWeight","bold");

startTime = tic;

%% NOISE PARAMETERS
NOISE_THRESH = 40;   % Weak peaks ignored
MIN_FREQ     = 5;    % Ignore < 5 Hz
MAX_FREQ     = 800;  % Good for walking, toy cars

%% CUSTOM LOCAL PEAK DETECTOR
find_local_peaks = @(x, thresh, mindist) ...
    local_peak_detector(x, thresh, mindist);

%% MAIN LOOP
while toc(startTime) < maxTime

    rawBlock = [];

    %% WAIT FOR START
    line = "";
    while ~strcmp(line,"START")
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));
        end
    end

    %% READ 512 SAMPLES
    for i = 1:FFT_SIZE
        rawLine     = strtrim(readline(s));
        rawBlock(i) = str2double(rawLine);
    end

    % Read END
    readline(s);

    %% PROCESSING
    x = rawBlock(:)' - mean(rawBlock);  % DC removal

    % LIGHT HIGH-PASS FILTER
    x = x - movmean(x, 30);

    rmsVal = rms(double(x));

    % MANUAL HAMMING WINDOW
    n  = 0:FFT_SIZE-1;
    w  = 0.54 - 0.46*cos(2*pi*n/(FFT_SIZE-1));
    xw = x .* w;

    % FFT
    X  = abs(fft(xw));
    X  = X(1:FFT_SIZE/2);

    % SMOOTH FFT
    Xs = smoothdata(X, "movmean", 5);

    %% CUSTOM PEAK DETECTION
    [pk_vals, pk_locs] = find_local_peaks(Xs, NOISE_THRESH, 4);

    if isempty(pk_vals)
        dopplerHz = 0;
        peakIndex = NaN;
    else
        [~, idx]  = max(pk_vals);
        peakIndex = pk_locs(idx);
        dopplerHz = freqAxis(peakIndex);

        if dopplerHz < MIN_FREQ || dopplerHz > MAX_FREQ
            dopplerHz = 0;
        end
    end

    % Convert to speed
    speed_kmh = dopplerHz * 0.0514;

    %% TIME
    t             = toc(startTime);
    tVec(end+1)   = t;
    fVec(end+1)   = dopplerHz;
    vVec(end+1)   = speed_kmh;
    rmsVec(end+1) = rmsVal;

    %% UPDATE PLOTS (BEAUTIFUL)

    % Time domain
    set(hTime,'XData',1:FFT_SIZE,'YData',x);

    % FFT
    set(hFFT,'XData',freqAxis,'YData',Xs);

    if dopplerHz > 0 && ~isnan(peakIndex)
        set(hPeak,'XData',dopplerHz,'YData',Xs(peakIndex));
    else
        set(hPeak,'XData',NaN,'YData',NaN);
    end

    % Doppler & Speed
    set(hFreq,'XData',tVec,'YData',fVec);
    set(hSpeed,'XData',tVec,'YData',vVec);

    drawnow limitrate;
end

%% SAVE RESULTS
dataLog.time      = tVec;
dataLog.frequency = fVec;
dataLog.speed     = vVec;
dataLog.signal    = rmsVec;

save("radar_data_clean_fft.mat","dataLog");
disp("✓ Clean radar session saved to radar_data_clean_fft.mat");

% =========================================================================
% CUSTOM PEAK DETECTOR — TOOLBOX FREE
% =========================================================================
function [pk_vals, pk_locs] = local_peak_detector(x, thresh, minddist)

pk_vals = [];
pk_locs = [];
N       = length(x);

% find simple local peaks
for i = 2:N-1
    if x(i) > thresh && x(i) > x(i-1) && x(i) > x(i+1)
        pk_vals(end+1) = x(i);
        pk_locs(end+1) = i;
    end
end

if isempty(pk_locs)
    return;
end

% Enforce minimum spacing
valid_vals = [];
valid_locs = [];
last_loc   = -inf;

for i = 1:length(pk_locs)
    if pk_locs(i) - last_loc >= minddist
        valid_vals(end+1) = pk_vals(i);
        valid_locs(end+1) = pk_locs(i);
        last_loc          = pk_locs(i);
    end
end

pk_vals = valid_vals;
pk_locs = valid_locs;

end
