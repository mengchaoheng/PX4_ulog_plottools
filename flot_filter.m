clear all;
close all;
clc;
addpath(genpath(pwd));
% you can run on terminal 
% ulog2csv log_.ulg 
% to get csv files
% =====================1==========================
% Install pyulog using pip first.https://github.com/PX4/pyulog.
% in MacOS, it maybe have been installed by the px4-dev
% =====================2==========================
% Make sure it has installed ulog2csv correctly (check the output of which ulog2csv in Linux/MacOS or where ulog2csv in Windows).
% =====================3==========================
% Change the following line in ulogviewver.m:
% command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg'];
% to 
% command = ['!your ulog2csv path' ulgFileName '.ulg'];
% and 
% ulgFileName = '00_41_22'; 
% to 
% ulgFileName = 'your log name (full path)'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%------------------------------------------
% Set ULog relative path
%------------------------------------------
ulgFileName = 'data/02_24_44';
tmp = [ulgFileName '.mat'];

% Record the current main script path
rootDir = fileparts(mfilename('fullpath'));

%------------------------------------------
% Step 1: Check if MAT file already exists
%------------------------------------------
if exist(fullfile(rootDir, tmp), "file")
    disp(['Found MAT file: ' tmp]);
    load(fullfile(rootDir, tmp), 'log');

else
    disp('No MAT file found, start parsing ULog...');

    %------------------------------------------
    % Step 2: Run ulog2csv (keep full path)
    %------------------------------------------
    if ismac
        ulog2csv_path = '~/Library/Python/3.9/bin/ulog2csv';
    else
        ulog2csv_path = 'ulog2csv';
    end

    ulgAbs = fullfile(rootDir, [ulgFileName '.ulg']);
    command = ['!' ulog2csv_path ' ' '"' ulgAbs '"'];
    disp(['Running command: ' command]);
    eval(command);

    %------------------------------------------
    % Step 3: Call parsing function (pass full path)
    %------------------------------------------
    log.data = csv_topics_to_d(fullfile(rootDir, ulgFileName));
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';

    %------------------------------------------
    % Step 4: Save MAT file to the same directory
    %------------------------------------------
    save(fullfile(rootDir, tmp), 'log');
    disp(['Saved MAT file: ' tmp]);

    %------------------------------------------
    % Step 5: Delete temporary CSV files
    %------------------------------------------
    delete(fullfile(rootDir, [ulgFileName '_*.csv']));
    disp('Temporary CSV files deleted.');
end
%%



if(isfield(log.data, 'allocation_value_0'))
    allocation_value=log.data.allocation_value_0{:,:};

end 
if(isfield(log.data, 'actuator_outputs_value_0'))
    actuator_outputs_value=log.data.actuator_outputs_value_0{:,:};

end 



if(isfield(log.data, 'actuator_outputs_value_0') && isfield(log.data, 'allocation_value_0'))
    % plot((allocation_value(:,1))*1e-6, allocation_value(:,12),'r-','LineWidth',1);hold on;
    plot((allocation_value(:,1))*1e-6, allocation_value(:,16),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot((actuator_outputs_value(:,1))*1e-6, actuator_outputs_value(:,2),'b--','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf -0.5 0.5]);
    title('Actuator dynamics');
    xlabel({'Time (s)'});
    ylabel('Response')
    legend('cmd','response','add filter for the same');
end 


