inputName = 'short4.wav';
outputName = 'short9.c';

fid = fopen(outputName, 'w'); %open file to write in binary format
if(fid == -1) 
    fprintf('ERROR: Could not open file \n');
    return;
end

[x, fs] = audioread(inputName, 'native'); %read file
y = uint8(x'); %transpose data to read into C easier

fprintf(fid, "static const short musicData[] = {");
fprintf(fid,'0x%.2x, ', y(:));
fprintf(fid, "}; \r\n");
fclose(fid);

return;