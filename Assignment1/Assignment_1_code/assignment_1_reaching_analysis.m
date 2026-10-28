%% Assignment 1 - MCI reaching task: analysis
%
% The pipeline, step by step:
%   Part 1  raw EMG -> your own envelopes (conventional filtering),
%           compared with the envelopes the MCI computed live
%   Part 2  the MCI map: muscles -> cursor velocity (given in the .mat)
%   Part 3  rebuild the cursor from the envelopes and compare it with the
%           logged one
%   Part 4  performance per reach
%   BONUS   drive a simulated 2-link robot arm with your cursor
%
% Input: your group's .mat file. It contains:
%   centerout          table, one row per sample: t, raw_<muscle>,
%                      env_<muscle>, cursor_x_px, cursor_y_px, cursor_x_cm, cursor_y_cm
%   centerout_targets  table, one row per target: target_x_px, target_y_px,
%                      spawn_time, hit_time, reach_time, overshoots
%   random, random_targets   the same for the random task
%   info               sampling frequency, muscles, screen, calibration, MCI map
% (double-click a table in the Workspace to see it like a spreadsheet)
%
% Toolbox needed: Signal Processing Toolbox (butter, filtfilt, xcorr).
%
% Wherever you see "% YOUR CODE HERE", complete the code. Keep the variable
% names written in the instructions: the code given after them uses those names.

clear; close all; clc;

%% 0. Settings
% -------------------------------------------------------------------------
%[folderPath, ~, ~] = fileparts(mfilename('fullpath'));
sessionPath = fileparts(mfilename('fullpath'));
sessionPath = fullfile(sessionPath, 'ID_02_Session_4');
make_group_mat(sessionPath, 'Cometa', 'Group_6.mat');

matFile = 'Group_6.mat';   % your group's data file
task    = 'centerout';     % which task to analyze: 'centerout' or 'random'
fsDown  = 100;             % rate we use after the envelope is computed [Hz]

animateCursor     = true;  % Part 3: animate the logged and the rebuilt cursor on the screen
resetAtEachTarget = true;  % Part 3: restart the rebuilt cursor from the logged one at each new target


%% 1. Load the data
% -------------------------------------------------------------------------
% The .mat already contains only the game part of the recording, with a
% regular time vector starting at 0. (In the original txt files the time
% column is uneven, because samples reach the PC in bursts, but there are
% exactly 2000 samples per second, so the time was rebuilt from the
% sample index.)
load(matFile);                        % info, centerout, centerout_targets, random, random_targets

% Pick the task we analyze
if strcmp(task, 'centerout')
    D = centerout;          targets = centerout_targets;
else
    D = random;             targets = random_targets;
end

fs       = info.fs;                   % EMG sampling frequency [Hz]
muscles  = info.muscles;              % 'R biceps', 'L biceps', ...
screenPx = info.screen_px;            % screen resolution [px]
cmPerPx  = info.cm_per_px;            % [cm per px]
centerPx = info.center_px;            % center target [px]
MAX      = info.calib_max;            % value of each muscle that the MCI calls 100%

% The table has one column per muscle (e.g. raw_R_biceps). For filtering
% it is handier to have the 4 muscles side by side in one matrix, so take
% those columns out of the table by name (column order = info.muscles).
muscleCols = strrep(muscles, ' ', '_');               % 'R biceps' -> 'R_biceps'

% Extract these variables from the table D and from targets:
%   t         time [s]                                     (N x 1)
%   raw       raw EMG, one column per muscle               (N x 4)
%   envMCI   MCI envelopes, one column per muscle (0-1)  (N x 4)
%   cursorPx  cursor position [px], y grows DOWNWARD       (N x 2)
%   cursorCm  cursor position [cm]                         (N x 2)
%   targetPx  target positions [px], one row per target    (nTargets x 2)
% Hint: D{:, strcat('raw_', muscleCols)} takes the 4 raw columns at once.

% YOUR CODE HERE
t = D{:,strcat('t')};
raw = D{:, strcat('raw_', muscleCols)};
envMCI = D{:, strcat('env_', muscleCols)};
cursorPx = D{:, strcat("cursor_", ["x","y"], "_px")};
cursorCm = D{:, strcat("cursor_", ["x","y"], "_cm")};
targetPx = targets{:, strcat("target_", ["x","y"], "_px")};


% Units: the raw EMG is stored in ADC counts. The sensors use a signed
% 16-bit converter (-32768 ... +32767) over an input range of +-2.5 mV, so
% one count is 2500 uV / 32768 = 0.0763 uV. We convert to microvolts so the
% plots have physical units. MAX comes from the same raw signal, so it is
% converted too (the normalized envelopes, 0-1, don't change).
uVperCount = 2500 / 32768;            % [uV per ADC count]
raw = raw * uVperCount;               % raw EMG [uV]
MAX = MAX * uVperCount;               % calibration max [uV]

N = numel(t);
fprintf('Loaded %s, %s task: %d samples = %.1f s, %d targets\n', ...
        info.system, task, N, t(end), height(targets));
fprintf('Muscles: %s\n', strjoin(muscles, ', '));


%% PART 1 - Preprocessing (conventional pipeline, as in Assignment 1)
% -------------------------------------------------------------------------
% 1) Band-pass 30-450 Hz: keeps the EMG band, removes movement artifacts
%    and offset (<30 Hz) and high-frequency noise (>450 Hz).
% 2) Rectify: EMG goes positive and negative around zero; its size (how
%    hard the muscle works) is in the absolute value.
% 3) Low-pass 5 Hz: smooths the rectified signal into an envelope that
%    follows the muscle activation, not the single spikes.
% 4) Normalize by MAX: puts every muscle on the same 0-1 scale, like the MCI.
%
% We use filtfilt (forward + backward): it has NO delay. The MCI works
% live, so it can only use past samples and its envelope has a delay.
% This is the main difference you will see in the comparison below.
%
% Steps (all 4 muscles at once: filtfilt works column by column):
%   a) design the band-pass and the low-pass filters
%   b) remove the DC offset from raw (subtract the mean of each column)
%   c) band-pass             -> emgFilt  (N x 4) [uV]
%   d) rectify               -> emgRect  (N x 4) [uV]
%   e) low-pass              -> envOurs  (N x 4) [uV]
%   f) normalize: divide each column by its MAX  -> envOurs (0-1)
%   g) filtering can dip slightly below 0: set negative values to 0

% YOUR CODE HERE

% Filter design
[bBand, aBand] = butter(4, [30 450] / (fs / 2), 'bandpass');
[bLow, aLow] = butter(4, 5 / (fs / 2), 'low');

% DC offset removal
raw = raw - mean(raw);

% Filtering steps
emgFilt = filtfilt(bBand, aBand, raw);
emgRect = abs(emgFilt);
envOursUV = filtfilt(bLow, aLow, emgRect);

% Normalization
envOursNorm = envOursUV ./ MAX;  

% Setting post-filtering negative values to 0 
envOursNorm(envOursNorm < 0) = 0;

% --- Figure 1: the preprocessing steps for each muscle, on a 3 s window
% with some activity (easier to see). For each muscle plot:
%   - raw and filtered EMG [uV]
%   - rectified EMG with the envelope on top [uV]
% Label the axes with units.

t0 = t(15000);
windowIdx = t >= t0 & t <= t0 + 3;

figure(1)
tiledlayout(4, 2, 'TileSpacing', 'compact')
for i = 1:4
    % Left column: raw and filtered EMG
    nexttile(2*i - 1)
    plot(t(windowIdx), raw(windowIdx, i))
    hold on
    plot(t(windowIdx), emgFilt(windowIdx, i))
    title(strcat('Raw and filtered - ', muscles{i}))
    xlabel('t [s]')
    ylabel('EMG [\muV]')
    legend('Raw signal', 'Filtered signal')

    % Right column: rectified EMG and envelope
    nexttile(2*i)
    plot(t(windowIdx), emgRect(windowIdx, i))
    hold on
    plot(t(windowIdx), envOursUV(windowIdx, i))
    title(strcat('Rectified and envelope - ', muscles{i}))
    xlabel('t [s]')
    ylabel('EMG [\muV]')
    legend('Rectified EMG', 'Envelope')
end 


% --- Compare your envelopes with the MCI ones (envMCI)
% For each muscle:
%   - correlation between the two envelopes (corrcoef): same shape?
%   - delay between them: the lag of the peak of xcorr (convert it to ms)
% Then plot the two envelopes on top of each other (a 30 s window is enough).

% YOUR CODE HERE

% Correlation matrices
corrMatrices = cell(1,4);
for i = 1:4
    R = corrcoef(envMCI(:,i), envOursNorm(:, i));
    corrMatrices{i} = R;
end

% Delay
delayValues = cell(1,4);
for i = 1:4
    [crossCorrelation, lags] = xcorr(envOursNorm(:,i) - mean(envOursNorm(:,i)), ...
                                      envMCI(:,i) - mean(envMCI(:,i)), 'coeff');
    [~, peakIndex] = max(crossCorrelation);
    delayValues{i} = lags(peakIndex) / fs * 1000; % delay in ms
end

% Envelope plots
t1 = t(15000);
windowIdx2 = t >= t1 & t <= t1 + 30;

figure(2)
tiledlayout(4,1)
for i = 1:4
    nexttile
    plot(t(windowIdx2), envMCI(windowIdx2,i))
    hold on 
    plot(t(windowIdx2), envOursNorm(windowIdx2,i))
    title(strcat('Envelope plots for muscle: ', muscles{i}))
    xlabel("s")
    legend("envMCI", "envOursNorm")
end 

% --- Downsample to 100 Hz
% WHY only now: the envelope is a slow signal (below 5 Hz), so 100 Hz is
% plenty. Doing it BEFORE filtering would alias the EMG (which goes up to
% 450 Hz) and destroy it. The cursor also only updates ~60 times per second.
step = fs / fsDown;                        % keep 1 sample every 20
dt   = 1 / fsDown;                         % time between downsampled samples [s]
% Keep one sample every 'step' and create:
%   tD, envOursD, envMCID, cursorCmD

% YOUR CODE HERE
tD = downsample(t, step);
envOursD = downsample(envOursNorm, step);
envMCID = downsample(envMCI, step);
cursorCmD = downsample(cursorCm, step);



%% PART 2 - The MCI map (given)
% -------------------------------------------------------------------------
% The MCI is in velocity control: the muscles set the cursor SPEED.
% Each axis is driven by a pair of muscles, and the speed is proportional
% to their DIFFERENCE, so contracting both together cancels out:
%
%     x speed = gain * (muscle A - muscle B)      (biceps pair)
%     y speed = gain * (muscle C - muscle D)      (trapezius pair)
%
% gain = Max Cursor Speed (50 cm/s): a muscle at 100% with the other one
% relaxed moves the cursor at 50 cm/s. The .mat stores the map (info.W) as a
% matrix W, one row per axis and one column per muscle, so that for all
% samples at once:
%
%     velocity (N x 2) = gain * envelopes (N x 4) * W'
gain = info.gain_cm_s;                     % [cm/s]
W    = info.W;                             % 2 x 4
fprintf('\nPart 2 - MCI map\n');
fprintf('  x speed        = %g * (%s)\n', gain, info.map_x);
fprintf('  y speed (down) = %g * (%s)\n', gain, info.map_y);

% Dead zone: the MCI ignores any muscle below 5% of its max, so that
% resting noise does not move the cursor. Without this step the rebuilt
% cursor creeps away from the real one even when the subject is relaxed.
deadZone = 0.05;

% 1) Apply the dead zone to envOursD and to envMCID (values below
%    deadZone become 0)
% 2) Compute the cursor velocity [cm/s] with the formula above:
%      velOurs  from your envelopes      (N x 2)
%      velMCI  from the MCI envelopes  (N x 2)

% YOUR CODE HERE

% Dead zone application
for i = 1:4
    envOursD(envOursD(:,i) < deadZone, i) = 0;
    envMCID(envMCID(:,i) < deadZone, i) = 0;
end

% Cursor velocity calculation
velOurs = gain*envOursD*W';
velMCI = gain*envMCID*W';


%% PART 3 - Rebuild the cursor from the envelopes
% -------------------------------------------------------------------------
% Velocity from the map (Part 2), then integrate (position = previous position +
% velocity * dt). Like the real cursor, ours can't leave the screen, so we
% clip it at the edges at every step.
%
% Integrating adds up every small error, so the rebuilt cursor slowly
% drifts away from the real one (try resetAtEachTarget = false to see it).
% With the reset, we restart from the logged position at each new target,
% so the error only builds up within one reach.
screenCm = info.screen_cm;                       % screen size [cm]
spawnT   = targets.spawn_time;                   % when each target appears [s]

% Rebuild two cursors, posOurs and posMCI [cm] (nD x 2, nD = numel(tD)),
% from velOurs and velMCI:
%   1) start both from the logged position: cursorCmD(1,:)
%   2) for each sample k = 2..nD: previous position + velocity * dt
%   3) clip to the screen: between [0 0] and screenCm
%   4) if resetAtEachTarget is true: at the first sample after each
%      spawnT, set both positions back to the logged one
% Then compute, for both, the distance from the logged cursor at each
% sample (errOurs, errMCI [cm]) and print its mean.

% YOUR CODE HERE
posOurs = zeros(numel(tD), 2);
posMCI = zeros(numel(tD), 2);
posOurs(1, :) = cursorCmD(1, :);
posMCI(1, :) = cursorCmD(1, :);

for k = 2:numel(tD)

    % Position update
    posOurs(k, :) = posOurs(k-1, :) + velOurs(k, :)*dt;
    posMCI(k, :) = posMCI(k-1, :) + velMCI(k, :)*dt;
    
    % Clipping control
    posOurs(k, :) = max(posOurs(k,:), [0, 0]);
    posMCI(k, :) = max(posMCI(k,:), [0, 0]);

    posOurs(k, :) = min(posOurs(k, :), screenCm);
    posMCI(k, :) = min(posMCI(k, :), screenCm);

    % Position reset
    if resetAtEachTarget
        resetIdx = find(tD(k) >= spawnT, 1, 'last');
        if resetIdx > 0 && tD(k-1) < spawnT(resetIdx)
            posOurs(k,:) = cursorCmD(k,:);
            posMCI(k,:) = cursorCmD(k,:);
        end
    end

end

errOurs = vecnorm(posOurs - cursorCmD, 2, 2);
errMCI = vecnorm(posMCI - cursorCmD, 2, 2);
fprintf('Mean cursor error: ours = %.3f cm, MCI = %.3f cm\n', mean(errOurs), mean(errMCI));


% Targets, converted to cm -> targCm (nTargets x 2)

% YOUR CODE HERE
targCm = targetPx .* (screenCm ./ info.screen_px);

% --- Animation: the cursor moving on the screen, target by target (given)
% Black = the real (logged) cursor, red = the one we rebuilt from the EMG.
% The current target is the green circle. It plays in real time.
if animateCursor
    nD = numel(tD);
    % which target is on screen at each sample = the last one spawned
    curTarget = zeros(nD, 1);
    for s = 1:numel(spawnT)
        curTarget(tD >= spawnT(s)) = s;
    end

    figure('Name', 'Part 3 - cursor animation');
    rectangle('Position', [0 0 screenCm]); hold on;            % the screen
    hTarg = plot(nan, nan, 'go', 'MarkerSize', 25, 'LineWidth', 2);
    hLogTrail = plot(nan, nan, 'Color', [0.6 0.6 0.6]);
    hLog  = plot(nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    hOurs = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    set(gca, 'YDir', 'reverse');                               % screen y grows downward
    axis equal; xlim([0 screenCm(1)]); ylim([0 screenCm(2)]);
    xlabel('x [cm]'); ylabel('y [cm]');
    legend([hLog hOurs hTarg], {'logged cursor', 'rebuilt cursor', 'target'}, 'Location', 'eastoutside');
    tic; k = 1;
    while k <= nD
        if curTarget(k) > 0
            set(hTarg, 'XData', targCm(curTarget(k),1), 'YData', targCm(curTarget(k),2));
        end
        k0 = max(1, k - 200);              % trail = last 2 s of the logged cursor
        set(hLogTrail, 'XData', cursorCmD(k0:k,1), 'YData', cursorCmD(k0:k,2));
        set(hLog,  'XData', cursorCmD(k,1), 'YData', cursorCmD(k,2));
        set(hOurs, 'XData', posOurs(k,1),   'YData', posOurs(k,2));
        title(sprintf('t = %.1f s   target %d', tD(k), curTarget(k)));
        drawnow;
        k = max(k + 1, round(toc * fsDown) + 1);   % jump to the sample of "now"
        pause(max(0, (k - 1) / fsDown - toc));     % ahead of the clock? wait
    end
end



%% PART 4 - Performance per reach
% -------------------------------------------------------------------------
% Each trial goes from the moment the target appears (spawn_time) to the
% moment it is hit (hit_time). In center-out, every other target is the
% center (the way back): we look at the outward reaches only. In the
% random task all targets count.
%   1) select the reaches to use: target hit (hit_time is not NaN) and
%      target not at the center (targetPx differs from centerPx)
%   2) for each of them, plot in order (a learning curve):
%      - the reach time [s]            (targets.reach_time)
%      - the overshoots [count]        (targets.overshoots)

% YOUR CODE HERE

isHit = ~isnan([targets.hit_time]);
targetPositions = [[targets.target_x_px] [targets.target_y_px]];
isOutward = any(targetPositions ~= centerPx, 2);
selectedTargets = targets(isHit & isOutward, :);
    
reachTimes = selectedTargets.reach_time;
overshoots = selectedTargets.overshoots;
figure;
subplot(2,1,1); plot(reachTimes, 'o-'); ylabel('Reach time [s]'); grid on;
subplot(2,1,2); plot(overshoots, 'o-'); ylabel('Overshoots [count]'); xlabel('Reach'); grid on;

%% PART 5 - Robot reaching: drive a 2-link robot arm with your cursor
% -------------------------------------------------------------------------
% Picture a planar arm with two links in front of the screen: the shoulder
% is at the origin, the screen center is 0.40 m in front of it. The screen
% is shrunk to 0.40 m wide, so that every point of it is within reach of
% the arm, whatever the size of the monitor used in the recording. The
% rebuilt cursor tells the HAND where to go; the robot needs JOINT angles
% q = [q1 q2] (shoulder, elbow). The Jacobian J links the two speeds:
%
%       hand velocity = J(q) * joint velocity
%
% so at every step we invert it to get the joint velocity, and integrate it
% to get the angles (resolved-rate control):
%
%       joint velocity = J(q) \ hand velocity
%
% Formulas you need (s1 = sin(q1), c1 = cos(q1), s12 = sin(q1+q2), c12 = cos(q1+q2)):
%   forward kinematics (hand position):
%       x = L1*c1 + L2*c12,   y = L1*s1 + L2*s12
%   Jacobian:
%       J = [ -L1*s1 - L2*s12 , -L2*s12 ;
%              L1*c1 + L2*c12 ,  L2*c12 ]
%   inverse kinematics (only for the first point, "elbow down"):
%       q2 = acos((x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2))
%       q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2))

L1 = 0.30;  L2 = 0.30;                     % link lengths [m]
centerRobot = [0 0.40];                    % screen center in the robot frame [m]
scale = 0.40 / (screenCm(1) / 100);        % screen -> robot: the screen becomes 0.40 m wide
Kcorr = 10;                                % correction gain [1/s], see step 4
nD = numel(tD);

% 1) Screen frame -> robot frame. From posOurs [cm, y down] compute the
%    hand reference handRef [m, y up] (nD x 2):
%      - subtract the screen center in cm (centerPx * cmPerPx)
%      - convert cm -> m and multiply by scale
%      - flip y (screen y grows downward, robot y grows upward)
%      - add centerRobot
%    Do the same for the targets: targCm -> targRobot (nTargets x 2).

% YOUR CODE HERE


% 2) Hand velocity handVel [m/s] (nD x 2): derivative of handRef
%    (gradient of each column, divided by dt).

% YOUR CODE HERE


% 3) Starting angles: inverse kinematics of the first point handRef(1,:)
%    (formulas above). Store them in q(1,:), with q = zeros(nD, 2).

% YOUR CODE HERE


% 4) For each sample k = 1..nD:
%      - compute the hand position with forward kinematics -> handArm(k,:)
%      - compute J at q(k,:) and its determinant -> detJ(k)
%      - if k < nD: joint velocity with the Jacobian, then integrate:
%            qdot = J \ (handVel(k,:) + Kcorr*(handRef(k,:) - handArm(k,:)))'
%            q(k+1,:) = q(k,:) + qdot' * dt
%    The Kcorr term pulls the hand back toward the reference: without it,
%    small integration errors (and the jumps when the cursor is reset at a
%    new target) would never be undone.

% YOUR CODE HERE


% 5) Check and plot:
%      - distance between handArm and handRef [mm]: print its mean and max
%      - the smallest |detJ|: near 0 = singularity (arm fully stretched)
%      - figure: hand path with the targets, and the joint angles [deg] in time

% YOUR CODE HERE


% --- Animation of the arm (given): plays in real time
if animateArm
    figure('Name', 'Part 5 - robot arm animation');
    plot(targRobot(:,1), targRobot(:,2), 'go', 'MarkerSize', 10, 'LineWidth', 2); hold on;
    hArm  = plot([0 0 0], [0 0 0], 'b-o', 'LineWidth', 3);
    hPath = plot(nan, nan, 'r');
    axis equal; axis([-0.35 0.35 -0.05 0.65]); xlabel('x [m]'); ylabel('y [m]');
    tic; k = 1;
    while k <= nD
        elbow = [L1*cos(q(k,1)), L1*sin(q(k,1))];
        set(hArm,  'XData', [0 elbow(1) handArm(k,1)], 'YData', [0 elbow(2) handArm(k,2)]);
        set(hPath, 'XData', handArm(1:k,1), 'YData', handArm(1:k,2));
        title(sprintf('t = %.1f s', tD(k)));
        drawnow;
        k = max(k + 1, round(toc * fsDown) + 1);   % jump to the sample of "now"
        pause(max(0, (k - 1) / fsDown - toc));     % ahead of the clock? wait
    end
end