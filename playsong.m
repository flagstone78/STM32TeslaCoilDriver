function s=playsong(inputName, outputPort)
c = onCleanup(@stopSong);
musicBufferSize = 10*64;
%instrreset; %resets all instruments


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

%y = uint8(x'); %transpose data to read into C easier

s = serial(outputPort);
set(s, 'BaudRate', 2000000);
set(s, 'OutputBufferSize', musicBufferSize);
fopen(s);

%flush buffer
while(s.BytesAvailable ~= 0)
    o = fread(s,s.BytesAvailable); %read
end

%fprintf(s,'a');
debug = 0;
fwrite(s, 'a', 'uint8');
for n = 1:musicBufferSize:(length(x)-musicBufferSize)
    %wait for signal
    while(s.BytesAvailable == 0) 
    end
    o = fread(s,s.BytesAvailable); %read
    %char(o)
    
    %send next 1000 bytes of audio
    fwrite(s, x(n:n+musicBufferSize-1), 'uint8');
    %fwrite(s, y(:), 'uint8');
    debug = debug+1 %counter to track transmission
end

%send shutdown signal
fwrite(s, 'stop', 'uint8');

fwrite(s, 'stop', 'uint8');
fclose(s);
return;

%graceful exit handler

function stopSong()
    fwrite(s, 'stop', 'uint8');
    fclose(s);
    delete(s);
    clear s
end

end