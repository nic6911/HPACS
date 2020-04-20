A = load("Regular_PID.txt");
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/60)*max(A))-100;
time10=(find(A>((60-A(1))*0.1)+A(1),1));
time90=(find(A>((60-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Risetime: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[60 60],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:68*60)-60)>0.6)));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settlingtime: %.1f min',settlingtime);
legend('Temperature','SP=60\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 75])
title('Classic PID')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
xlim([0 150])
%%
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('Classic PID','-dpng','-r900')

ylim([50 70])
yticks([50 52 54 56 58 60 62 64 66 68 70])
print('Classic PID Zoom','-dpng','-r900')
%%
A = load("PI.txt");
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((max(A)-60)/60)*100;
time10=(find(A>((60-A(1))*0.1)+A(1),1));
time90=(find(A>((60-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Risetime: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[60 60],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:80*60)-60)>0.6)));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settlingtime: %.1f min',settlingtime);
legend('Temperature','SP=60\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 75])
title('Classic PI')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
xlim([0 150])
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('Classic PI','-dpng','-r900')

ylim([50 70])
yticks([50 52 54 56 58 60 62 64 66 68 70])
print('Classic PI Zoom','-dpng','-r900')
%%
