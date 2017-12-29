# takes the matrix P and shuffle it by columns

function [randP] = shuffleP(P)
idx = randperm(length(P));
randP = P(:,idx);
endfunction 