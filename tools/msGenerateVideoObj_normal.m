%% msGenerateVideoObj_normal.m
% Input:
%      videoFiles: a n*1 strings. consists of the paths of videos;

% Created by Xiang Zhang, 2021.

function ms = msGenerateVideoObj_normal(videoFiles)
    
    ms.dirName = fileparts(videoFiles{1});
    ms.vidName = videoFiles;
    ms.numFiles = 0;
    ms.numFrames = 0;
    ms.vidNum = [];
    ms.frameNum = [];
    ms.maxFramesPerFile = 0;
    ms.numFiles = length(videoFiles);
    
    time_start = 0;

    for i = 1:ms.numFiles
        ms.vidObj{i} = VideoReader(videoFiles{i});
        ms.vidNum = [ms.vidNum i*ones(1,ms.vidObj{i}.NumberOfFrames)];
        ms.frameNum = [ms.frameNum 1:ms.vidObj{i}.NumberOfFrames];
        
        % time;
        for j = 1:ms.vidObj{i}.NumberOfFrames
            frame = ms.vidObj{i}.read(j); %#ok<NASGU>
            ms.time(ms.numFrames + j, 1) = time_start + ms.vidObj{i}.CurrentTime;
        end
        
        ms.numFrames = ms.numFrames + ms.vidObj{i}.NumberOfFrames;
        ms.maxFramesPerFile = max(ms.maxFramesPerFile, ms.vidObj{i}.NumberOfFrames);
        time_start = ms.time(end);
    end

    ms.height = ms.vidObj{1}.Height;
    ms.width = ms.vidObj{1}.Width;
    
end