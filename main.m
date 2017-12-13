clear  
clc
global vm vt xnt  hedge  xnp
vm=3000; vt=1000; xnt=2; hedge=-10; xnp=5;
rm1_0=0; rm2_0=10000; rt1_0=40000; rt2_0=10000; beta=0;

vt1=-vt*cos(beta);
vt2=vt*sin(beta);

he=hedge/57.3;

rtm1_0=rt1_0-rm1_0;
rtm2_0=rt2_0-rm2_0;
xlam_0=atan(rtm2_0/rtm1_0);
xlead_0=asin(vt*sin(beta+xlam_0)/vm);
thet_0=xlam_0+xlead_0;
vm1_0=vm*cos(thet_0+he);
vm2_0=vm*sin(thet_0+he);

tf=11;
delta=0.011;
t=0:delta:tf; 
x(:,1)=[0;rt1_0;rt2_0;vm1_0;vm2_0;rm1_0;rm2_0];
for i=1:(length(t)-1)
    k1=delta*motion(i*delta,x(:,i));
    k2=delta*motion(i*delta+delta/2,x(:,i)+k1/2);
    k3=delta*motion(i*delta+delta/2,x(:,i)+k2/2);
    k4=delta*motion(i*delta+delta/2,x(:,i)+k3);
    x(:,i+1)=x(:,i)+1/6*(k1+2*k2+2*k3+k4);
    
    rtm1_0=x(2,i)-x(6,i);
    rtm2_0=x(3,i)-x(7,i);
    rtm=sqrt(rtm1_0*rtm1_0+rtm2_0*rtm2_0);
    vtm1=(-vt*cos(x(1,i)))-x(4,i);
    vtm2=vt*sin(x(1,i))-x(5,i);
    vc=-( rtm1_0*vtm1+rtm2_0*vtm2)/rtm;
    
    if vc<0
        tbl=rtm;
        break;
    end
end

rt1=x(2,:);
rt2=x(3,:);
rm1=x(6,:);
rm2=x(7,:);
figure(1)
a=plot(rt1,rt2,rm1,rm2);
hold on;