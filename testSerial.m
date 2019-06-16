inputName = 'justshapes8bit20kHz.wav';
outputPort = 'COM10';
instrreset; %resets all instruments

%read in data
[x, fs] = audioread(inputName, 'native'); %read file
if(class(x) ~= 'uint8')
    fprintf('ERROR: Audio must be unsigned 8 bit data with 20kHz sampling rate\n');
    return;
end

if(fs ~= 20000)
    fprintf('ERROR: Audio must be unsigned 8 bit data with 20kHz sampling rate\n');
    return;
end

s = serial(outputPort);
set(s, 'BaudRate', 2000000);
set(s, 'OutputBufferSize', 1000);
fopen(s);
%flush buffer
while(s.BytesAvailable ~= 0)
    o = fread(s,s.BytesAvailable); %read
end

%fprintf(s,'a');
fwrite(s, 'a', 'uint8');
bufferSize = 1000;

y = mod(1:1000, 256);
%wait for signal
while(s.BytesAvailable == 0) 
end
o = fread(s,s.BytesAvailable); %read
char(o)

%send next 1000 bytes of audio
fwrite(s, y(:), 'uint8');

while(s.BytesAvailable == 0) 
end
o = fread(s,s.BytesAvailable); %read
char(o)
fwrite(s, y(:), 'uint8');

fclose(s);
delete(s);
clear s;
return;