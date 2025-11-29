function vec2csvfile(filename, vec)
% VEC2CSVFILE Generates .csv file with data from single-precision input
% vector in hexadecimal representation of raw memory content
%   VEC2CSVFILE(FILENAME, VEC) generate FILENAME text file with data form
%   VEC vector
%
%   Example:
%        vec2csvfile('mydata.csv', [1.0, 2.0, 3.0])
%
%   generates mydata.csv file with following content:
%        0x3F800000,
%        0x40000000,
%        0x40400000
%
    fileID = fopen(filename,'w');
    for i = 1 : length(vec)-1
        fprintf(fileID,'%s,\n', typeConv_float_to_hex(vec(i)));
    end
    fprintf(fileID,'%s\n',  typeConv_float_to_hex(vec(end)));
    fclose(fileID);
end

function hex_str = typeConv_float_to_hex(f_val)
    f = single(f_val);
    byte_array = typecast(f, 'uint32');
    hex_str = dec2hex(byte_array, 8);
    hex_str = hex_str(:)';
    hex_str = ['0x' hex_str];
end