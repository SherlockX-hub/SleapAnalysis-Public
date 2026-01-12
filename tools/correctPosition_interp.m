%% correctPosition.m
% This code is used to correct the position of recording, which has been
% used in the DLC position data.
%
% Input:
%       pos_origial: the original data;
%       behav: the behav struct data;
%       isInterp: whether do the interpolation;
% Output:
%       pos: the corrected position data;
%       pos2: the interpolated position data of 'pos', no nan value;
%       pos3: the filtered position data of 'pos';
%       roi: the region of position;
%
% Created by Xiang Zhang, 2021.

function [pos, pos2, pos3, roi] = correctPosition_interp(pos_original, behav, isInterp)
    if nargin < 3, isInterp = 0; end

    if size(pos_original,2) ~= 2
        error('Wrong input of position data.');
    end
    
    % roi;
    if ~isfield(behav, 'ROI')
        if isfield(behav, 'vidObj')
            frame = behav.vidObj{1}.read(1);
            if size(frame,3) == 4, frame = frame(:,:,1:3); end
            imshow(frame,'InitialMagnification','fit');
        end
        hold on;
        plot(pos_original(:,1),pos_original(:,2));
        hold off;
        axis equal;
        roi = drawrectangle;
        pause;
        roi = roi.Position;
        behav.ROI = roi;
        close();
    else
        roi = behav.ROI;
    end
    
    % position correction;
    if isscalar(behav.trackLength), behav.trackLength = [behav.trackLength behav.trackLength]; end
    pos_original(:,1) = (pos_original(:,1) - behav.ROI(1)) / behav.ROI(3) * behav.trackLength(1);
    pos_original(:,2) = (pos_original(:,2) - behav.ROI(2)) / behav.ROI(4) * behav.trackLength(2);
    
    pos = pos_original;
    pos_center = [behav.trackLength(1) / 2, behav.trackLength(2) / 2];
    
    % remove the position out of boundary;
    for i = 1:size(pos,1)
        if pos(i,1) < -2 || pos(i,1) > behav.trackLength(1) + 2 || ...
                pos(i,2) < -2 || pos(i,2) > behav.trackLength(2) + 2 % boundary;
            pos(i,:) = NaN;
        end
        
        switch behav.shape
            case 2
                if norm(pos(i,:) - pos_center) > behav.trackLength(1) / 2 + 2.5 % circular boundary;
                    pos(i,:) = NaN;
                end
            case 3
                if norm(pos(i,:) - pos_center) < behav.radiusRange(1) - 2 || ...
                        norm(pos(i,:) - pos_center) > behav.radiusRange(2) + 2 % annular boundary;
                    pos(i,:) = NaN;
                end
        end
    end
    
    % remove too fast speed;
    figure;
    hold on;
    if ~isfield(behav, 'time')
        speed = vecnorm(pos(3:end,:) - pos(1:end-2,:), 2, 2) ./ (3:size(pos,1) - 1:size(pos,1)-2)';
        speed = [speed(1); speed; speed(end)];
        % speed = speed2D(pos(:,1), pos(:,2), 1:size(pos,1));
    else
        speed = vecnorm(pos(3:end,:) - pos(1:end-2,:), 2, 2) ./ (behav.time(3:end) - behav.time(1:end-2));
        speed = [speed(1); speed; speed(end)];
        % speed = speed2D(pos(:,1), pos(:,2), behav.time);
    end
    plot(speed);
    
    threshold = input('Enter the threshold of speed threshold: ');
    if isempty(threshold)
        threshold = 100; % cm/s;
    end
    close all;
    pos((speed > threshold),:) = NaN;
    
    % remove wrong position between nan values;
    ind = find(isnan(pos(:,1)));
    ind2 = find(~isnan(pos(:,1)));
    pos1 = pos;
    for ind_i = 1:length(ind) % fix small gaps (nan);
        previousNotNaN = ind2(find(ind2 < ind(ind_i), 1, 'last'));
        nextNotNaN = ind2(find(ind2 > ind(ind_i), 1));
        if isempty(previousNotNaN) || isempty(nextNotNaN) || ...
                previousNotNaN < 3 || nextNotNaN > size(pos,1) - 2
            continue;
        end
        if nextNotNaN - previousNotNaN <= 5
            if abs(pos(previousNotNaN,1) - pos(nextNotNaN,1)) < 25 && ...
                    abs(pos(previousNotNaN,2) - pos(nextNotNaN,2)) < 25
                pos1(ind(ind_i),:) = mean(pos(ind(ind_i)-2: ind(ind_i)+2,:), 'omitnan');
            end
        end
    end
    
    % delete a list of positions;
    ind = find(isnan(pos1(:,1)));
    ind2 = find(~isnan(pos1(:,1)));
    for ind_i = 1:length(ind) - 1
        if ind(ind_i) ~= 1 && ind(ind_i+1) - ind(ind_i) <= 30 && ind(ind_i+1) - ind(ind_i) > 1
            previousNotNaN = ind2(find(ind2 < ind(ind_i), 1, 'last'));
            nextNotNaN = ind2(find(ind2 > ind(ind_i+1), 1));
            if isempty(previousNotNaN) || isempty(nextNotNaN)
                continue;
            end
            if nextNotNaN - previousNotNaN <= 30
                if (abs(pos(previousNotNaN,1) - pos(ind(ind_i)+1,1)) > 25 || ...
                        abs(pos(previousNotNaN,2) - pos(ind(ind_i)+1,2)) > 25) && ...
                        (abs(pos(nextNotNaN,1) - pos(ind(ind_i+1)-1,1)) > 25 || ...
                        abs(pos(nextNotNaN,2) - pos(ind(ind_i+1)-1,2)) > 25)
                    pos(ind(ind_i):ind(ind_i+1),:) = nan;
                    ind_i = ind_i+1; %#ok<FXSET,NASGU>
                end
            end
        end
    end
    ind_all = sum(isnan(pos(:,1)));

    % interpolation nan to value;
    if ~isInterp
        pos2 = pos;
    else
        ind2 = find(~isnan(pos(:,1)));
        pos2 = pos;
        if ~isfield(behav, 'time')
            pos2(ind2(1):ind2(end), :) = interp1(ind2', pos(ind2,:), (ind2(1):ind2(end))', 'pchip');
        else
            pos2(ind2(1):ind2(end), :) = interp1(behav.time(ind2), pos(ind2,:), behav.time(ind2(1):ind2(end)), 'pchip');
        end
    end
    
    % median filter;
    if  behav.shape == 3 % annular;
        pos3 = smoothCircPos(pos, behav.trackLength(1));
        
        % set too fast point to nan;
        speed = speed2D(pos3(:,1), pos3(:,2), behav.time);
        pos3((speed > 0.7),:) = NaN;
    else
        pos3 = smoothdata(pos,'movmedian',15,'omitnan');
    end
    
    plot(pos3(:,1),pos3(:,2));
    axis equal;
    
    disp(['The trajectory has ',num2str(ind_all),' nan values']);
    pause;
    close all;
end

%% functions;
function pos = smoothCircPos(pos, diameter)
    pos_distance = sqrt(sum((pos - diameter / 2).^2,2));
    pos_angle = atan2(pos(:,2) - diameter / 2, pos(:,1) - diameter / 2);
    
    pos_distance = smoothdata(pos_distance, 'movmedian', 15, 'omitnan');
    pos_angle_rotation = zeros(length(pos_angle),1);
    for i = 2:length(pos_angle)
        pos_angle_rotation(i) = angleDiffer(pos_angle(i), pos_angle(i-1), 'rad');
    end
    LEDRotation_threshold = 30 * pi / 180;
    pos_angle(pos_angle_rotation > LEDRotation_threshold) = NaN;
    pos_angle = angleSmooth(pos_angle, 'rad', 'movmedian', 15, 0);
    
    pos(:,1) = diameter / 2 + pos_distance .* cos(pos_angle);
    pos(:,2) = diameter / 2 + pos_distance .* sin(pos_angle);
end