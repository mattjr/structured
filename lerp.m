r=[0:0.01:1];
Cb=[0.5 ,0.1,0.2];
j=1; 
rmax=0.70710678;
 
num=exp(-5*((r-Cb(j))/(rmax*Cb(j))));
W=num./(1+num);
plot(r,W);
grid on; axis([0 1 0 1]);