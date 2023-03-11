samplen=1000;%Total number of initial states
InitialMat=zeros(4,samplen);%Store initialized x, y, theta, v into each row by sequence
for i=1:samplen
rr=0;
while rr<=1
xini1=(-10+rand(1)*(20));%-10~10
yini1=(-10+rand(1)*(20));%-10~10
rr = (xini1)^2+(yini1)^2;%Initialize location states which are inside safe region
end
thetaini1=(-10+rand(1)*(20));%-10~10
vini1=(-10+rand(1)*(20));%-10~10
InitialMat(1,i)=xini1;
InitialMat(2,i)=yini1;
InitialMat(3,i)=thetaini1;
InitialMat(4,i)=vini1;
end
save('InitialStateData','InitialMat');