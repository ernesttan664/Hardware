interval = 430;
t=0;
dc = zeros(1,interval/10);
for i=1:interval/10
    
    dc(i) = sin(2.4*pi*t*0.001)*sin(2.4*pi*t*0.001);
    t=t+10;
end