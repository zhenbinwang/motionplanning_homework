clc; clear;
close all;


syms ts real;
for i=1:4
    for j=1:7+2-i
        M_k(i+4,j) = ts^(7+1-j+1-i);
    end
end