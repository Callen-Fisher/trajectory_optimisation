function [ F,G ] = userfunction(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    [obj,grad] = objective(x);

    F = [obj];
    G = [grad];
    
    
    [c,ceq,dc,dceq] = constraint(x);
    
    F = [ F; c; ceq];%concaternate the values 
    G = [ G; dc; dceq];%concaternate the gradients

end

