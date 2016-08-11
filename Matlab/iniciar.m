clear
close all
global h2 versor

versor =  [1 0 0
           0 1 0
           0 0 1];

hf2 = figure();
h2  = axes('Parent',hf2);
s = serial('/dev/ttyACM0');
set(s,'BaudRate',115200);
fopen(s);

t = timer('StartDelay',0);
set(t,'ExecutionMode','fixedSpacing','Period',0.02)
set(t,'TimerFcn',{@evento,s},'StopFcn','fclose(s);')
start(t)