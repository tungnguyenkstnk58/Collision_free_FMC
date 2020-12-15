clear
clc
close all

%%
t_start  = 2;
t_end    = 10;
Ts = 5e-2; Tsim = 20; Ns = round(Tsim/Ts);
x = zeros(Ns,1);
for k = 1:1:Ns
    x(k) = 10 * step_fcn(k, t_start/Ts, t_end/Ts);
end

plot(Ts:Ts:Tsim,x);


function output = step_fcn(mr, low, up)
    if mr <= low
        output = 0;
    elseif mr >= up
        output = 1;
    else
        output = (mr - low)^3/(up -low)^3;
    end
end