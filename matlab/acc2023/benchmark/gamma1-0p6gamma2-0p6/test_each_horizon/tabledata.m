nmpcdata=zeros(3,6);%Store data in matrix for nmpc and impc, first row: mean of computing time; second row: standard deviation of computing time; third row: infeasible rate
impcdata=zeros(3,6);
for ii = 1:6
i=ii*4;
filenm1 = ['timecom' num2str(i) '.mat'];
filenm2 = ['feasibility' num2str(i) '.mat'];
load(filenm1);
load(filenm2);
distnmpc = fitdist(nmpcplot1','Normal');
distimpc = fitdist(impcplot1','Normal');
mu1=distnmpc.mu;%Get mean of sample of computing time for NMPC-DCBF
mu2=distimpc.mu;%Get mean of sample of computing time for iMPC-DCBF
sigma1=distnmpc.sigma;%Get variance of sample of computing time for NMPC-DCBF
sigma2=distimpc.sigma;%Get variance of sample of computing time for iMPC-DCBF
nmpcdata(1,ii)=mu1;
impcdata(1,ii)=mu2;
nmpcdata(2,ii)=sigma1;
impcdata(2,ii)=sigma2;
nmpcdata(3,ii)=nmpcinf;
impcdata(3,ii)=impcinf;
end