function dotState=motion(t,state)
global vt xnt   xnp
dotState = zeros(7,1);
%列写微分方程如下：
dotState(1)=xnt/vt;
dotState(2)= -vt*cos(state(1));
dotState(3)= vt*sin(state(1));

rtm1=state(2)-state(6);
rtm2=state(3)-state(7);

rtm=sqrt(rtm1*rtm1+rtm2*rtm2);
vtm1=(-vt*cos(state(1)))-state(4);
vtm2=vt*sin(state(1))-state(5);
vc=-( rtm1*vtm1+rtm2*vtm2)/rtm;
dlbt=(rtm1*vtm2-rtm2*vtm1)/(rtm*rtm);
nc=xnp*vc*dlbt;
xlam=atan((state(3)-state(7))/(state(2)-state(6)));
dotState(4)=nc*sin(xlam);
dotState(5)=nc*cos(xlam);
dotState(6)=state(4);
dotState(7)=state(5);
