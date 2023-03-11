close all
clear

%--------------------------------------------------------------------------------------
nsim=100;%Total time-steps for the trajectory
xo = 0;%Obstacle parameters
yo = 0;
r = 1;
N=24;%Number of iteration
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\trajectory');%For the location address, please change it later based on where you download the files
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\iteration_x');
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\iteration_y');
figure(1)%Figure 2-a of paper
set(0,'defaultfigurecolor','w'); 
set(gca,'LooseInset',get(gca,'TightInset'));
hold on;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(xo + r*x, yo + r*y, 'k-', 'LineWidth', 2);
fill(xo + r*x, yo + r*y, [1 0.5 0]);
axis square
plot(impctra(1,1), impctra(2,1), 'db', 'LineWidth', 1);
plot(3, 0, 'dr', 'LineWidth', 1);
h = plot(impctra(1,:), impctra(2,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
h1 = plot(Itera_x(1,:), Itera_y(1,:),'c', 'LineWidth', 1);%
h2 =plot(Itera_x(3,:), Itera_y(3,:),'g', 'LineWidth', 1);
h3 =plot(Itera_x(5,:), Itera_y(5,:),'m', 'LineWidth', 1);
h4 =plot(Itera_x(10,:), Itera_y(10,:),'r', 'LineWidth', 1);
h5 =plot(Itera_x(32,:), Itera_y(32,:),'b', 'LineWidth', 1);

h_legend = legend([h,h1,h2,h3,h4,h5], {'Closed-loop ($t_{\mathrm{sim}}=100$)','Open-loop ($j=1$)','Open-loop ($j=3$)','Open-loop ($j=5$)',...
    'Open-loop ($j=10$)','Open-loop ($j=32$)'}, 'Location', 'SouthEast');
set(h_legend, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 19);
grid on
axis equal
xlim([-3,3]);
ylim([-2,2]);
xlabel('$x(m)$','interpreter','latex','FontSize',20);
ylabel('$y(m)$','interpreter','latex','FontSize',20);
text('Interpreter','latex','String','$t=6$','Position',[-2.45 0.9],'FontSize',18);
annotation('arrow',[0.21 0.21],[0.70 0.60],'LineStyle', '-','color',[0 0 0]);
print(gcf,'figures/openloop-snapshots', '-depsc');
print(gcf,'figures/openloop-snapshots.png', '-dpng', '-r800');

load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N24_Gamma4_4');%impc-24-4-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N24_Gamma6_6');%impc-24-6-6
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N16_Gamma4_4');%impc-16-4-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N16_Gamma6_6');%impc-16-6-6
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N24_Gamma4_4');%nmpc-24-6-6
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N24_Gamma6_6');%nmpc-24-4-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N16_Gamma4_4');%nmpc-16-6-6
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N16_Gamma6_6');%nmpc-16-4-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N24_Gamma4');%impc-24-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\impc_N24_Gamma6');%impc-24-6
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N24_Gamma4');%nmpc-24-4
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\nmpc_N24_Gamma6');%nmpc-24-6

figure(2)%Figure 2-b of paper
%Draw the figure
xo = 0;
yo = 0;
r = 1;
xr = [3;0.01;0;0];
set(0,'defaultfigurecolor','w'); 
set(gca,'LooseInset',get(gca,'TightInset'));
hold on;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(xo + r*x, yo + r*y, 'k-', 'LineWidth', 2);
fill(xo + r*x, yo + r*y, [1 0.5 0]);
axis square
plot(impc1(1,1), impc2(2,1), 'db', 'LineWidth', 1);
plot(xr(1), xr(2), 'dr', 'LineWidth', 1);
h1 = plot(impc1(1,:), impc1(2,:), 'k',...
        'LineWidth', 2.0,'MarkerSize',4);
h2 = plot(impc2(1,:), impc2(2,:), 'r',...
        'LineWidth', 2.0,'MarkerSize',4);
h3 = plot(impc3(1,:), impc3(2,:), 'b',...
        'LineWidth', 2.0,'MarkerSize',4);
h4 = plot(impc4(1,:), impc4(2,:), 'm',...
        'LineWidth', 2.0,'MarkerSize',4);

h_legend1 = legend([h1,h2,h3,h4], {'$N=24,\gamma_{1}=0.4,\gamma_{2}=0.4$','$N=24,\gamma_{1}=0.6,\gamma_{2}=0.6$','$N=16,\gamma_{1}=0.4,\gamma_{2}=0.4$','$N=16,\gamma_{1}=0.6,\gamma_{2}=0.6$'}, 'Location', 'SouthEast');
set(h_legend1, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 18);
grid on
axis equal
xlim([-3,3]);
ylim([-2,2]);
xlabel('$x(m)$','interpreter','latex','FontSize',20);
ylabel('$y(m)$','interpreter','latex','FontSize',20);
print(gcf,'figures/closedloop-snapshots1', '-depsc');
print(gcf,'figures/closedloop-snapshots1.png', '-dpng', '-r800');

figure(3)%Figure 2-c of paper
% Draw the figure
xo = 0;
yo = 0;
r = 1;
xr = [3;0.01;0;0];
set(0,'defaultfigurecolor','w'); 
set(gca,'LooseInset',get(gca,'TightInset'));
hold on;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(xo + r*x, yo + r*y, 'k-', 'LineWidth', 2);
fill(xo + r*x, yo + r*y, [1 0.5 0]);
axis square
plot(impc1(1,1), impc2(2,1), 'db', 'LineWidth', 1);
plot(xr(1), xr(2), 'dr', 'LineWidth', 1);
h5 = plot(controller_nmpc_dcbf_multiple1(1,:), controller_nmpc_dcbf_multiple1(2,:), 'k--',...
        'LineWidth', 2.0,'MarkerSize',4);
h6 = plot(controller_nmpc_dcbf_multiple2(1,:), controller_nmpc_dcbf_multiple2(2,:), 'r--',...
        'LineWidth', 2.0,'MarkerSize',4);
h7 = plot(controller_nmpc_dcbf_multiple3(1,:), controller_nmpc_dcbf_multiple3(2,:), 'b--',...
        'LineWidth', 2.0,'MarkerSize',4);
h8 = plot(controller_nmpc_dcbf_multiple4(1,:), controller_nmpc_dcbf_multiple4(2,:), 'm--',...
        'LineWidth', 2.0,'MarkerSize',4);
h_legend2 = legend([h5,h6,h7,h8], {'$N=24,\gamma_{1}=0.4,\gamma_{2}=0.4$','$N=24,\gamma_{1}=0.6,\gamma_{2}=0.6$','$N=16,\gamma_{1}=0.4,\gamma_{2}=0.4$','$N=16,\gamma_{1}=0.6,\gamma_{2}=0.6$'}, 'Location', 'SouthEast');
set(h_legend2, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 16);
grid on
axis equal
xlim([-3,3]);
ylim([-2,2]);
xlabel('$x(m)$','interpreter','latex','FontSize',20);
ylabel('$y(m)$','interpreter','latex','FontSize',20);
text('Interpreter','latex','String','$t=13$','Position',[-2 -1.1],'FontSize',18);
annotation('arrow',[0.37 0.47],[0.33 0.38],'LineStyle', '-','color',[0 0 0]);
text('Interpreter','latex','String','$t=33$','Position',[1 0.1],'FontSize',18);
annotation('arrow',[0.78 0.88],[0.53 0.48],'LineStyle', '-','color',[0 0 0]);
print(gcf,'figures/closedloop-snapshots2', '-depsc');
print(gcf,'figures/closedloop-snapshots2.png', '-dpng', '-r800');

figure(4)%Figure 2-d of paper
% Draw the figure
xo = 0;
yo = 0;
r = 1;
xr = [3;0.01;0;0];
set(0,'defaultfigurecolor','w'); 
set(gca,'FontSize',20)
set(gca,'LooseInset',get(gca,'TightInset'));
hold on;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(xo + r*x, yo + r*y, 'k-', 'LineWidth', 2);
fill(xo + r*x, yo + r*y, [1 0.5 0]);
axis square
plot(impc1(1,1), impc2(2,1), 'db', 'LineWidth', 1);
plot(xr(1), xr(2), 'dr', 'LineWidth', 1);
h9 = plot(impc5(1,1:46), impc5(2,1:46), 'k',...
        'LineWidth', 2.0,'MarkerSize',4);
h10 = plot(impc6(1,1:46), impc6(2,1:46), 'r',...
        'LineWidth', 2.0,'MarkerSize',4);
h11 = plot(controller_nmpc_dcbf_multiple5(1,:), controller_nmpc_dcbf_multiple5(2,:), 'b--',...
        'LineWidth', 2.0,'MarkerSize',4);
h12 = plot(controller_nmpc_dcbf_multiple6(1,:), controller_nmpc_dcbf_multiple6(2,:), 'm--',...
        'LineWidth', 2.0,'MarkerSize',4);
h_legend3 = legend([h9,h10,h11,h12], {'$N=24,\gamma_{1}=0.4$','$N=24,\gamma_{1}=0.6$','$N=24,\gamma_{1}=0.4$','$N=24,\gamma_{1}=0.6$'}, 'Location', 'SouthEast');
set(h_legend3, 'Interpreter','latex');
set(gca,'LineWidth', 0.2, 'FontSize', 18);
grid on
axis equal
xlim([-3,3]);
ylim([-2,2]);
xlabel('$x(m)$','interpreter','latex','FontSize',20);
ylabel('$y(m)$','interpreter','latex','FontSize',20);
print(gcf,'figures/closedloop-snapshots3', '-depsc');
print(gcf,'figures/closedloop-snapshots3.png', '-dpng', '-r800');

load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\jtconv_Gamma4_4');
figure(5)%Figure 4-a of paper
kk = 0:1:(nsim-1);
hk = plot(kk, kk44(1,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
xlim([0,99]);
ylim([0,1000]);
set(gca,'FontSize',18)
xlabel('$t$','interpreter','latex','FontSize',20);
ylabel('$j_{t,\mathrm{conv}}$','interpreter','latex','FontSize',20);
set(gca, 'YScale', 'log')
print(gcf,'figures/J-converge-1', '-depsc');
print(gcf,'figures/J-converge-1.png', '-dpng', '-r400');

load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\jtconv_Gamma4_6'); 

figure(6)%Figure 4-b of paper
kk = 0:1:(nsim-1);
hk = plot(kk, kk46(1,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
xlim([0,99]);
ylim([0,1000]);
xlabel('$t$','interpreter','latex','FontSize',20);
ylabel('$j_{t,\mathrm{conv}}$','interpreter','latex','FontSize',20);
set(gca,'FontSize',18)
set(gca, 'YScale', 'log')
print(gcf,'figures/J-converge-2', '-depsc');
print(gcf,'figures/J-converge-2.png', '-dpng', '-r400');


load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\jtconv_Gamma6_4');

figure(7)%Figure 4-c of paper
kk = 0:1:(nsim-1);
hk = plot(kk, kk64(1,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
xlim([0,99]);
xlabel('$t$','interpreter','latex','FontSize',20);
ylabel('$j_{t,\mathrm{conv}}$','interpreter','latex','FontSize',20);
set(gca,'FontSize',18)
set(gca, 'YScale', 'log')
print(gcf,'figures/J-converge-3', '-depsc');
print(gcf,'figures/J-converge-3.png', '-dpng', '-r400');

load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\jtconv_Gamma6_6');

figure(8)%Figure 4-d of paper
kk = 0:1:(nsim-1);
hk = plot(kk, kk66(1,:), 'ko-',...
        'LineWidth', 1.0,'MarkerSize',4);
xlim([0,99]);
xlabel('$t$','interpreter','latex','FontSize',20);
ylabel('$j_{t,\mathrm{conv}}$','interpreter','latex','FontSize',20);
set(gca,'FontSize',18)
set(gca, 'YScale', 'log')
print(gcf,'figures/J-converge-4', '-depsc');
print(gcf,'figures/J-converge-4.png', '-dpng', '-r400');

 
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\iteration_theta');
load('D:\onedri\OneDrive\Desktop\shuo\Github\Figure2_3_4\iteration_v');

figure(9)%Figure 3-a of paper
kk1 = 0:1:N;
hh = plot(kk1, Itera_x(32,:),'r', 'LineWidth', 1);
hh1 = plot(kk1, Itera_x(1,:),'r', 'LineWidth', 1);
hold on 
for i=2:1:32
if i<=20
hh1= plot(kk1, Itera_x(i,:),'r', 'LineWidth', 1);
else
hh=plot(kk1, Itera_x(i,:),'b', 'LineWidth', 2);
hold on 
xlim([0,24]);
end
end
hh_legend = legend([hh,hh1], {'Converged ($j=32$)','Converging ($1\le j<32$)'}, 'Location', 'SouthEast');
set(hh_legend, 'Interpreter','latex','FontSize',18);
text('Interpreter','latex','String','$j=1$','Position',[19 0.55],'FontSize',18);
annotation('arrow',[0.75 0.7],[0.62 0.71],'LineStyle', '-','color',[0 0 0]);
text('Interpreter','latex','String','$j=2$','Position',[5.3 0.6],'FontSize',18);
annotation('arrow',[0.3 0.4],[0.6 0.55],'LineStyle', '-','color',[0 0 0]);
set(gca,'FontSize',18)
xlabel('$k$','interpreter','latex','FontSize',20);
ylabel('$x(m)$','interpreter','latex','FontSize',20);
print(gcf,'figures/state-converge-1', '-depsc');
print(gcf,'figures/state-converge-1.png', '-dpng', '-r400');
 
figure(10)%Figure 3-b of paper
kk1 = 0:1:N;
hh = plot(kk1, Itera_y(32,:),'r', 'LineWidth', 1);
hh1 = plot(kk1, Itera_y(1,:),'r', 'LineWidth', 1);
hold on 
for i=2:1:32
if i<=20
hh1= plot(kk1, Itera_y(i,:),'r', 'LineWidth', 1);
else
hh=plot(kk1, Itera_y(i,:),'b', 'LineWidth', 2);
hold on 
xlim([0,24]);
end
end
hh_legend = legend([hh,hh1], {'Converged ($j=32$)','Converging ($1\le j<32$)'}, 'Location', 'SouthEast');
set(hh_legend, 'Interpreter','latex','FontSize',18);
text('Interpreter','latex','String','$j=1$','Position',[16.5 1],'FontSize',18);
annotation('arrow',[0.7 0.75],[0.75 0.66],'LineStyle', '-','color',[0 0 0]);
text('Interpreter','latex','String','$j=2$','Position',[1.9 0.90],'FontSize',18);
annotation('arrow',[0.2 0.27],[0.68 0.6],'LineStyle', '-','color',[0 0 0]);
set(gca,'FontSize',18)
xlabel('$k$','interpreter','latex','FontSize',20);
ylabel('$y(m)$','interpreter','latex','FontSize',20);
print(gcf,'figures/state-converge-2', '-depsc');
print(gcf,'figures/state-converge-2.png', '-dpng', '-r400');


figure(11)%Figure 3-c of paper
kk1 = 0:1:N;
hh = plot(kk1, Itera_theta(32,:),'r', 'LineWidth', 1);
hh1 = plot(kk1, Itera_theta(1,:),'r', 'LineWidth', 1);
hold on 
for i=2:1:32
if i<=20
hh1= plot(kk1, Itera_theta(i,:),'r', 'LineWidth', 1);
else
hh=plot(kk1, Itera_theta(i,:),'b', 'LineWidth', 2);
hold on 
xlim([0,24]);
end
end
hh_legend = legend([hh,hh1], {'Converged ($j=32$)','Converging ($1\le j<32$)'}, 'Location', 'SouthEast');
set(hh_legend, 'Interpreter','latex','FontSize',18);
text('Interpreter','latex','String','$j=1$','Position',[13.5 0],'FontSize',18);
annotation('arrow',[0.58 0.48],[0.55 0.5],'LineStyle', '-','color',[0 0 0]);
text('Interpreter','latex','String','$j=2$','Position',[5.4 -0.17],'FontSize',18);
annotation('arrow',[0.37 0.44],[0.43 0.35],'LineStyle', '-','color',[0 0 0]);
set(gca,'FontSize',18)
xlabel('$k$','interpreter','latex','FontSize',20);
ylabel('$\theta(rad)$','interpreter','latex','FontSize',20);
print(gcf,'figures/state-converge-3', '-depsc');
print(gcf,'figures/state-converge-3.png', '-dpng', '-r400');

 
figure(12)%Figure 3-d of paper
kk1 = 0:1:N;
hh = plot(kk1, Itera_v(32,:),'r', 'LineWidth', 1);
hh1 = plot(kk1, Itera_v(1,:),'r', 'LineWidth', 1);
hold on 
for i=2:1:32
if i<=20
hh1= plot(kk1, Itera_v(i,:),'r', 'LineWidth', 1);
else
hh=plot(kk1, Itera_v(i,:),'b', 'LineWidth', 2);
hold on 
xlim([0,24]);
end
end
hh_legend = legend([hh,hh1], {'Converged ($j=32$)','Converging ($1\le j<32$)'}, 'Location', 'SouthEast');
set(hh_legend, 'Interpreter','latex','FontSize',18);
text('Interpreter','latex','String','$j=1$','Position',[17.5 1.7],'FontSize',18);
annotation('arrow',[0.74 0.64],[0.55 0.5],'LineStyle', '-','color',[0 0 0]);
text('Interpreter','latex','String','$j=2$','Position',[8.5 2.7],'FontSize',18);
annotation('arrow',[0.42 0.32],[0.83 0.88],'LineStyle', '-','color',[0 0 0]);
set(gca,'FontSize',18)
xlabel('$k$','interpreter','latex','FontSize',20);
ylabel('$v(m\cdot s^{-1})$','interpreter','latex','FontSize',20);
print(gcf,'figures/state-converge-4', '-depsc');
print(gcf,'figures/state-converge-4.png', '-dpng', '-r400');
