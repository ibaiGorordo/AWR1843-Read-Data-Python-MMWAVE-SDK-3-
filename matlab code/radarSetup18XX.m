function [DATA_sphandle,UART_sphandle, ConfigParameters] = radarSetup18XX(configfile)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%         CONFIGURE SERIAL PORT          %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% UART COM PORT:
comPortString = 'COM4';
UART_sphandle = serial(comPortString,'BaudRate',115200);
set(UART_sphandle,'Parity','none')
set(UART_sphandle,'Terminator','LF')
fopen(UART_sphandle);

%%%% DATA COM PORT:
comPortString = 'COM5';
DATA_sphandle = serial(comPortString,'BaudRate',921600);
set(DATA_sphandle,'Terminator', '');
set(DATA_sphandle,'InputBufferSize', 65536);
set(DATA_sphandle,'Timeout',10);
set(DATA_sphandle,'ErrorFcn',@dispError);
set(DATA_sphandle,'BytesAvailableFcnMode','byte');
set(DATA_sphandle,'BytesAvailableFcnCount', 2^16+1);%BYTES_AVAILABLE_FCN_CNT);
set(DATA_sphandle,'BytesAvailableFcn',@readUartCallbackFcn);
fopen(DATA_sphandle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%        READ CONFIGURATION FILE         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% configfile = "C:\Users\ibaig\Desktop\Carpeta comp\OneDrive\PHD\Radar\slowProfile.cfg";
config = cell(1,100);
fid = fopen(configfile, 'r');
if fid == -1
    fprintf('File %s not found!\n', configfile);
    return;
else
    fprintf('Opening configuration file %s ...\n', configfile);
end
tline = fgetl(fid);
k=1;
while ischar(tline)
    config{k} = tline;
    tline = fgetl(fid);
    k = k + 1;
end
config = config(1:k-1);
fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%       PARSE THE CONFIGURATION FILE         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:length(config)
    configLine = strsplit(config{i});
    
    % We are only interested in the channelCfg, profileCfg and frameCfg parameters:
    if strcmp(configLine{1},'channelCfg')
        channelCfg.txChannelEn = str2double(configLine{3});
        
        numTxAzimAnt = bitand(bitshift(channelCfg.txChannelEn,0),1) +...
                bitand(bitshift(channelCfg.txChannelEn,-2),1);
        numTxElevAnt = bitand(bitshift(channelCfg.txChannelEn,-1),1);
                
        channelCfg.rxChannelEn = str2double(configLine{2});
        numRxAnt = bitand(bitshift(channelCfg.rxChannelEn,0),1) +...
            bitand(bitshift(channelCfg.rxChannelEn,-1),1) +...
            bitand(bitshift(channelCfg.rxChannelEn,-2),1) +...
            bitand(bitshift(channelCfg.rxChannelEn,-3),1);
        numTxAnt = numTxElevAnt + numTxAzimAnt;
        
    elseif  strcmp(configLine{1},'profileCfg')
        profileCfg.startFreq = str2double(configLine{3});
        profileCfg.idleTime =  str2double(configLine{4});
        profileCfg.rampEndTime = str2double(configLine{6});
        profileCfg.freqSlopeConst = str2double(configLine{9});
        profileCfg.numAdcSamples = str2double(configLine{11});
        profileCfg.numAdcSamplesRoundTo2 = 1;
        while profileCfg.numAdcSamples > profileCfg.numAdcSamplesRoundTo2
            profileCfg.numAdcSamplesRoundTo2 = profileCfg.numAdcSamplesRoundTo2 * 2;
        end 
        profileCfg.digOutSampleRate = str2double(configLine{12}); %uints: ksps
        
    elseif strcmp(configLine{1},'frameCfg')
        frameCfg.chirpStartIdx = str2double(configLine{2});
        frameCfg.chirpEndIdx = str2double(configLine{3});
        frameCfg.numLoops = str2double(configLine{4});
        frameCfg.numFrames = str2double(configLine{5});
        frameCfg.framePeriodicity = str2double(configLine{6});
        
    end
end  


ConfigParameters.numChirpsPerFrame = (frameCfg.chirpEndIdx -...
    frameCfg.chirpStartIdx + 1) *...
    frameCfg.numLoops;
ConfigParameters.numDopplerBins = ConfigParameters.numChirpsPerFrame / numTxAnt;
ConfigParameters.numRangeBins = profileCfg.numAdcSamplesRoundTo2;
ConfigParameters.rangeResolutionMeters = 3e8 * profileCfg.digOutSampleRate * 1e3 /...
    (2 * profileCfg.freqSlopeConst * 1e12 * profileCfg.numAdcSamples);
ConfigParameters.rangeIdxToMeters = 3e8 * profileCfg.digOutSampleRate * 1e3 /...
    (2 * profileCfg.freqSlopeConst * 1e12 * ConfigParameters.numRangeBins);
ConfigParameters.dopplerResolutionMps = 3e8 / (2*profileCfg.startFreq*1e9 *...
    (profileCfg.idleTime + profileCfg.rampEndTime) *...
    1e-6 * ConfigParameters.numDopplerBins * numTxAnt);
ConfigParameters.maxRange = 300 * 0.9 * profileCfg.digOutSampleRate /(2 * profileCfg.freqSlopeConst * 1e3);
ConfigParameters.maxVelocity = 3e8 / (4*profileCfg.startFreq*1e9 *(profileCfg.idleTime + profileCfg.rampEndTime) * 1e-6 * numTxAnt);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%        SEND CONFIGURATION TO SENSOR         %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mmwDemoCliPrompt = char('mmwDemo:/>');

%Send CLI configuration to IWR14xx
fprintf('Sending configuration from %s file to IWR16xx ...\n', configfile);
for k=1:length(config)
    command = config{k};
    fprintf(UART_sphandle, command);
    fprintf('%s\n', command);
    echo = fgetl(UART_sphandle); % Get an echo of a command
    done = fgetl(UART_sphandle); % Get "Done"
    prompt = fread(UART_sphandle, size(mmwDemoCliPrompt,2)); % Get the prompt back
end


end