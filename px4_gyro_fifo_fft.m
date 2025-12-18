clear;
close all;
clc;
addpath(genpath(pwd));
% you can run on terminal 
% ulog2csv log_8_2021-5-20-11-52-08.ulg 
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
% ulgFileName = 'your log name'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%%
ulgFileName = '08_21_58'; % the ulog file name 
tmp=[ ulgFileName '.mat'];
% exist tmp var
if exist(tmp,"file")
    load(ulgFileName,'log');
else
    if ismac
        % on macOS, run " which ulog2csv " on terminal to get it.
        command = ['!/Users/mch/opt/anaconda3/bin/ulog2csv ' ulgFileName '.ulg']; % /usr/local/bin/ is the path of ulog2csv, 
    else
        % on windows and linux just make sure you have installed pyulog
        command = ['!ulog2csv ' ulgFileName '.ulg']; % have installed ulog2csv,
    end

	eval(command);
    log.data = csv_topics_to_d(ulgFileName);
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';
    %run add_fields_in_preprocessing.m
    save(ulgFileName,'log')
    delete(['*' ulgFileName '*.csv'])
end
%% 

sensor_gyro_fifo=log.data.sensor_gyro_fifo_0; %table
[t_new,xyz_new]=add_virtual_fifo_topic_data(sensor_gyro_fifo);


% rate_N=size(timestamp);
% rate_delta_t=zeros(rate_N-1);
% for i=1:rate_N-1
%   rate_delta_t(i)=(timestamp(i+1))*1e-6-(timestamp(i))*1e-6;
% end
L = size(t_new,1);   % Length of signal

T=(t_new(end)-t_new(1))*1e-6/L; % Sampling period  (second)
Fs=round(1/T); % Sampling frequency

t = (0:L-1)*T;        % Time vector
num = size(xyz_new,2);   % num of data type, like acc_x,acc_y,acc_z, then the num = 3
le_str = cell(num,1);  % for legen


%% 
% 
%% 
% Plot the noisy signal in the time domain. It is difficult to identify the 
% frequency components by looking at the signal |X(t)|. 


for i = 1:num
    plot((t_new-t_new(1))*1e-6,xyz_new(:,i));
    le_str{i} = ['X_new_',num2str(i)];
    hold on;
end

title('Signal')
xlabel('t (seconds)')
ylabel('X(t)')
legend(le_str,'Location','northeastoutside','box','off');





%% 
% Compute the Fourier transform of the signal. 

Y = fft(xyz_new);
%% 
% Compute the two-sided spectrum |P2|. Then compute the single-sided spectrum 
% |P1| based on |P2| and the even-valued signal length |L|.

P2 = abs(Y/L);
P1 = P2(1:L/2+1,:);
P1(2:end-1,:) = 2*P1(2:end-1,:);
%% 
% Define the frequency domain |f| and plot the single-sided amplitude spectrum 
% |P1|. The amplitudes are not exactly at 0.7 and 1, as expected, because of the 
% added noise. On average, longer signals produce better frequency approximations.
figure,
f = Fs*(0:(L/2))/L;
for i = 1:num
    plot(f,P1(:,i));
    le_str{i} = ['P1_',num2str(i)];
    hold on;
end
title('FFT of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
legend(le_str,'Location','northeastoutside','box','off');

%% Power spectral density under different parameters
%%
% figure,
% ss = spectrogram(X(:,1));
% spectrogram(X(:,1),'yaxis');
%%
% figure,
% Nx = length(X(:,1));
% nsc = floor(Nx/4.5);
% nov = floor(nsc/2);
% nff = max(256,2^nextpow2(nsc));
% 
% tt = spectrogram(X(:,1),hamming(nsc),nov,nff);
% spectrogram(X(:,1),hamming(nsc),nov,nff,Fs,'yaxis');
% maxerr = max(abs(abs(tt(:))-abs(ss(:))));
%%
% figure,
% 
% ns = 8;
% ov = 0.5;
% lsc = floor(Nx/(ns-(ns-1)*ov));
% 
% ttt = spectrogram(X(:,1),lsc,floor(ov*lsc),nff);
% spectrogram(X(:,1),lsc,floor(ov*lsc),nff,'yaxis');
% maxerr = max(abs(abs(ttt(:))-abs(ss(:))))
%%
% figure,
% spectrogram(X(:,1),256,128,256,Fs,'yaxis');
% h = gca;
% h.XTickLabel = string(h.XTick * 60);
% xlabel('Time (s)');
%%
Nw = 128;
window = hamming(256);
noverlap = 128;
nfft = 2^nextpow2(length(window));
fs = Fs;
%%
% figure,
% subplot(131)
% spectrogram(X(:,1), window, noverlap, nfft,fs, 'yaxis');  % Display the spectrogram
% title('Call directly')
% h=colorbar;
% h.Label.String = 'Power/Frequency(dB/Hz)'
% 
% [SS, FF, TT, PP] = spectrogram(X(:,1), window, noverlap, nfft,fs);
% subplot(132)
% imagesc(TT,FF,10*log10(PP));
% set(gca,'YDir','normal')
% title('Draw with P')
% h=colorbar;
% h.Label.String = 'Power/Frequency(dB/Hz)'
% 
% subplot(133)
% k = 2/(fs*(window'*window))
% imagesc(TT,FF,10*log10(abs(SS).*abs(SS)*k));
% set(gca,'YDir','normal')
% title('Draw with S')
% h=colorbar;
% h.Label.String = 'Power/Frequency(dB/Hz)'
%%
figure,
[SS1, FF1, TT1, PP1] = spectrogram(xyz_new(:,1), window, noverlap, nfft,fs);
[~, ~, ~, PP2] = spectrogram(xyz_new(:,2), window, noverlap, nfft,fs);
[~, ~, ~, PP3] = spectrogram(xyz_new(:,3), window, noverlap, nfft,fs);
PP=PP1+PP2+PP3;
imagesc(TT1,FF1,10*log10(PP));
set(gca,'YDir','normal')
% title('Power spectral density')
xlabel('Time t/s');
ylabel('Frequency f/Hz');
title('Short-time Fourier Transform Spectrogram');
h=colorbar;
h.Label.String = 'Power/Frequency(dB/Hz)'


