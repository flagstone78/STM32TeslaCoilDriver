outputName = 'test.wav';
inputName = 'short8.bin';
fs = 18000;

fid = fopen(inputName, 'r'); %open file to write in binary format
if(fid == -1) 
    fprintf('ERROR: Could not open file \n'); 
    return
end

x = fread(fid, inf, 'uint8'); %read file into a nchanxdim0 array
fclose(fid);

x = x';
soundsc(x,fs);
%audiowrite(outputName,x,fs);