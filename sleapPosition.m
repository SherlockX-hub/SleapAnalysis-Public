%% sleapPosition.m
% Modified from DLCPosition_v9_TwoPhoton.m.
%
% Created by Xiang Zhang, April, 2024.
% Normal version on December, 2025.

clear;

%% code path;
addpath(genpath('C:\Data\Dropbox\code\SleapAnalysis'));

%% parameters;
dir_name = 'C:\Data\Data\emotional_discrimination\CHD8_flox\20251217F1-1'; %pwd; % input('Enter the path of data: ', 's');
video_name = 'Mouse0001_20251218_09-33-16_R_unstressed.avi';
save_type = 1; % 0;
disp(['Start session: ', dir_name, filesep, video_name]);

%% create behav file;
if exist([dir_name filesep 'behav.mat'],'file')
    load([dir_name filesep 'behav.mat']);
else
    switch save_type
        case 0
            video_names = dir([dir_name, filesep, '*.avi']);
            video_name = {}; % wait to be done.
            behav = msGenerateVideoObj_normal(video_name);
        case 1
            behav = msGenerateVideoObj_normal({[dir_name, filesep, video_name]});
    end
end

%% collect all csv files;
for csv_i = 1:length(behav.vidName)
    [~, vid_name_temp] = fileparts(behav.vidName{csv_i});
    csv_temp = dir([dir_name, '/*', vid_name_temp, '.analysis.csv']);
    csv_temp = [dir_name, filesep, csv_temp(1).name];
    
    pos = readtable(csv_temp, 'VariableNamingRule','preserve');
    if csv_i == 1, pos_all = pos;
    else
        pos.frame_idx = pos.frame_idx + sum(behav.vidNum(behav.vidNum == (csv_i-1)));
        pos_all = [pos_all; pos]; %#ok<AGROW>
    end
end

sleap_position = pos_all;
switch save_type
    case 0
        save([dir_name filesep 'sleapPosition.mat'], 'sleap_position');
    case 1
        [~, video_name] = fileparts(video_name);
        save([dir_name filesep 'sleapPosition_', video_name, '.mat'], 'sleap_position');
end
disp('Original position data is collected and saved.');

%% separate mouse data;
if iscell(pos_all.track)
    mouse_mode = 0; % more than 1;
    indi_all = unique(pos_all.track);
else
    mouse_mode = 1;
    indi_all = {'mouse1'};
end
text_show = '';
if mouse_mode == 1
    pos_indi{1,1} = pos_all;
    text_show = ['mouse1 ', num2str(size(pos_indi{1,1},1)), '/', num2str(behav.numFrames), ' '];
else
    pos_indi = cell(length(indi_all),1);
    for indi_i = 1:length(indi_all)
        pos_indi{indi_i,1} = pos_all(strcmpi(pos_all.track, indi_all{indi_i}),:);
        text_show = [text_show, indi_all{indi_i}, ' ', num2str(size(pos_indi{indi_i,1},1)), '/', num2str(behav.numFrames), ' ']; %#ok<AGROW>
    end
    frame_indi = cellfun(@(x) size(x,1), pos_indi);
    frame_thre = 100;
    indi_all(frame_indi < frame_thre) = [];
    pos_indi(frame_indi < frame_thre) = [];
end
disp(text_show);

% interpolation;
for indi_i = 1:length(indi_all)
    pos_temp = pos_indi{indi_i};
    if size(pos_temp,1) ~= behav.numFrames
        pos_temp_temp = interp1(pos_temp.frame_idx, table2array(pos_temp(:,2:end)), 0:behav.numFrames-1);
        pos_temp = repmat(pos_temp(1,:), behav.numFrames,1);
        pos_temp(:,2:end) = array2table(pos_temp_temp);
        pos_indi{indi_i} = pos_temp;
    end
end

%% ROI;
frameIdx = round(behav.numFrames/2);
vidNum = behav.vidNum(frameIdx);
vidFrameNum = behav.frameNum(frameIdx);

% select ROI;
frame = behav.vidObj{vidNum}.read(vidFrameNum);
imshow(frame,'InitialMagnification','fit');
disp('Select ROI');
roi = drawrectangle;
pause;
behav.ROI = roi.Position;
close();

%% select DLC labels;
part_num = floor(size(pos_all,2)/3-1);
imshow(frame,'InitialMagnification','fit');
hold on;
scatter(table2array(pos_indi{1}(frameIdx, 4:3:size(pos_all,2))), ...
    table2array(pos_indi{1}(frameIdx, 5:3:size(pos_all,2))), [], 1:part_num, 'filled');
text(table2array(pos_indi{1}(frameIdx, 4:3:size(pos_all,2))), ...
    table2array(pos_indi{1}(frameIdx, 5:3:size(pos_all,2))), string(1:part_num), 'Color','w');
colormap jet;
colorbar;
sleap_part = input('Enter the number of body part you want to choose (i.e. [2,1,3], details in V9 description): ');
if isempty(sleap_part), sleap_part = 1:part_num; end
close();

%% correct head direction (optional);
LEDDir = input('Need calculate direction? (Y/N) ', 's');
LEDDir = strcmpi(LEDDir, 'Y');
if LEDDir
    dotNum = length(sleap_part);
    behav.correctionAngle = zeros(length(indi_all),1);
    
    LEDDir_correct = input('Need correct LED direction? (Y/N) ', 's');
    LEDDir_correct = strcmpi(LEDDir_correct, 'Y');
    if LEDDir_correct
        jet_color = flipud(jet);
        for indi_i = 1:length(indi_all)
            frameIdx = 1;
            userInput = 'Y';
            while(strcmpi(userInput, 'Y'))
                frameIdx = frameIdx + 500;
                vidNum = behav.vidNum(frameIdx);
                vidFrameNum = behav.frameNum(frameIdx);
                frame = behav.vidObj{vidNum}.read(vidFrameNum);
                if size(frame,3) == 4, frame = frame(:,:,1:3); end
                
                imshow(frame,'InitialMagnification','fit');
                hold on;
                scatter(pos_all(frameIdx,3*sleap_part(1)+4), pos_all(frameIdx,3*sleap_part(1)+5), 100,jet_color(1,:),'filled');
                scatter(pos_all(frameIdx,3*sleap_part(2)+4), pos_all(frameIdx,3*sleap_part(2)+5), 100,jet_color(end,:),'filled');
                hold off;
                figure(1);
                userInput = upper(input('Need another frame? (Y/N) ','s'));
            end
            
            % draw lines;
            disp('Draw a line of LED (red to blue direction)');
            lineLED = drawline; pause;
            vectorLED = diff(lineLED.Position);
            
            disp('Draw a line of mouse direction');
            lineMouse = drawline; pause;
            vectorMouse = diff(lineMouse.Position);
            
            hold off;
            close();
            
            behav.correctionAngle(indi_i,1) = mod(atan2d(det([vectorLED;vectorMouse]),dot(vectorLED,vectorMouse)),360);
        end
    end
end

%% parameters;
behav.trackLength = input('Enter the size (length or diameter) of experimental area: ');
behav.shape = input('Enter the shape of experimental area (1: box, 2: round, 3: annular): ');
if isscalar(behav.trackLength), behav.trackLength = [behav.trackLength behav.trackLength]; end
switch behav.shape
    case 1
        behav.radiusRange = [];
    case 2
        behav.radiusRange = behav.trackLength / 2;
    case 3
        innerLength = input('Enter the diameter of the inner experimental area: ');
        behav.radiusRange = [innerLength / 2 behav.trackLength / 2];
end

switch save_type
    case 0
        save([behav.dirName filesep 'behav.mat'], 'behav');
    case 1
        save([behav.dirName filesep 'behav_', video_name, '.mat'], 'behav');
end

%% correct DLC position;
disp('Start correction...');

score_thre = 0.5;
for indi_i = 1:length(indi_all)
    pos_temp = pos_indi{indi_i};
    pos = table2array(pos_temp(:,sleap_part(1)*3+4:sleap_part(1)*3+6));
    pos(pos(:,3) < score_thre, 1:2) = nan;
    pos = pos(:, 1:2);
    [~, pos] = correctPosition_interp(pos, behav, 1);
    behav.position{indi_i,1} = pos;
    behav.position_smooth{indi_i,1} = smoothdata(pos,'movmedian',15,'omitnan');
    behav.speed{indi_i,1} = speed2D(pos(:,1), pos(:,2), behav.time);
    behav.coverage{indi_i,1} = boxCoverage(pos(:,1), pos(:,2), 2, behav.shape, behav.radiusRange);
    
    % head direction correction;
    if LEDDir
        hdDir = atan2(table2array(pos_temp(:, 3*sleap_part(2)+5)) - table2array(pos_temp(:, 3*sleap_part(1)+5)), ...
            table2array(pos_temp(:, 3*sleap_part(2)+4)) - table2array(pos_temp(:, 3*sleap_part(1)+4)));
        hdDir = mod(360 * hdDir / (2*pi) + behav.correctionAngle(indi_i), 360);
        [behav.originalAngle{indi_i,1}, corredtedAngle] = deal(hdDir);
        
        % remove too long LED bar;
        LEDBar = vecnorm(table2array(pos_temp(:, 3*sleap_part(1)+4 : 3*sleap_part(1)+5)) - ...
            table2array(pos_temp(:, 3*sleap_part(2)+4 : 3*sleap_part(2)+5)), 2, 2);
        plot(LEDBar);
        LEDBar_threshold = input('Enter a threshold for LED bar length: ');
        close();
        if isempty(LEDBar_threshold), LEDBar_threshold = 30; end
        corredtedAngle(LEDBar > LEDBar_threshold) = NaN;
        
        behav.hdDir{indi_i,1} = correctAngle(corredtedAngle);
        behav.hdDir_smooth{indi_i,1} = angleSmooth(behav.hdDir{indi_i,1}, 'deg', 'movmedian', 15, 0);
    end
end

%% save figures and files;
figure;
for fig_i = 1:length(behav.position)
    subplot(1, length(behav.position), fig_i);
    plot(behav.position{fig_i}(:,1), behav.position{fig_i}(:,2));
    axis equal;
    axis([0 behav.trackLength(1) 0 behav.trackLength(2)]);
end
switch save_type
    case 0
        saveas(gcf, [behav.dirName filesep 'behav_position.png']);
    case 1
        saveas(gcf, [behav.dirName filesep 'behav_position_', video_name, '.png']);
end
close all;

behav = rmfield(behav, 'vidObj');
switch save_type
    case 0
        save([behav.dirName filesep 'behav.mat'], 'behav');
    case 1
        save([behav.dirName filesep 'behav_', video_name, '.mat'], 'behav');
end

disp([dir_name, filesep, video_name, ' All data are corrected and saved.']);
