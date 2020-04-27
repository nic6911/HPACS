%%
A = load("4LTuning.txt");
timevect = 0:1:length(A)-1;
timevect=timevect./60;
plot(timevect,A)
hold on
plot([0 timevect(end)],[55 55],'--','LineWidth',1.25)
plot([152.5 152.5],[48 62],'--','LineWidth',1.25)
xlabel('Time [min]')
ylabel('Temperature [\circC]')
grid on
title('Automatic Tuning')
legend('Temperature','Tuning reference','Tuning done')
txt = 'Tu = 1796';
text(110,52,txt)
txt = 'Ku = 23.282';
text(110,51.4,txt)
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('Autotune','-dpng','-r900')
close all
%%
A = load("PID_57_5_4l.txt");
SP=57.5;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/SP)*max(A))-100;
time10=(find(A>((SP-A(1))*0.1)+A(1),1));
time90=(find(A>((SP-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Rise time: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:100*60)-SP)>(SP*0.02))));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settling time: %.1f min',settlingtime);
legend('Temperature','SP=57.5\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 60])
title('Classic PID')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('PID_57_5','-dpng','-r900')
close all
%%
A = load("PI_55_4l.txt");
SP=55;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/SP)*max(A))-100;
time10=(find(A>((SP-A(1))*0.1)+A(1),1));
time90=(find(A>((SP-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Rise time: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:161*60)-SP)>(SP*0.02))));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settling time: %.1f min',settlingtime);
legend('Temperature','SP=55\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 65])
title('Classic PI')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('PI_55','-dpng','-r900')
close all
%%
A = load("PID_55_4l.txt");
SP=55;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/SP)*max(A))-100;
time10=(find(A>((SP-A(1))*0.1)+A(1),1));
time90=(find(A>((SP-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Rise time: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:66*60)-SP)>(SP*0.02))));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settling time: %.1f min',settlingtime);
legend('Temperature','SP=55\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 60])
title('Classic PID')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('PID_55','-dpng','-r900')
close all
%%
A = load("NOOS_55_4l.txt");
SP=55;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/SP)*max(A))-100;
time10=(find(A>((SP-A(1))*0.1)+A(1),1));
time90=(find(A>((SP-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Rise time: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:85*60)-SP)>(SP*0.02))));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settling time: %.1f min',settlingtime);
legend('Temperature','SP=55\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 60])
title('No Overshoot PID')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('NOOS_55','-dpng','-r900')
close all
%%
A = load("Pessen_55_4l.txt");
SP=55;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
overshoot = ((100/SP)*max(A))-100;
time10=(find(A>((SP-A(1))*0.1)+A(1),1));
time90=(find(A>((SP-A(1))*0.9)+A(1),1));
risetime = timevect(time90)-timevect(time10);
risetimetext = sprintf('Rise time: %.1f min',risetime);
overshoottext = sprintf('Peak: %.1f %cC, Overshoot: %.1f %%',max(A),char(176),overshoot);
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
p = plot([timevect(time10) timevect(time10)],[A(time10) A(time10)],'.','LineWidth',1,'Color',[0,0,0]);
plot([timevect(time90) timevect(time90)],[A(time90) A(time90)],'.','LineWidth',1,'Color',[0,0,0])
set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
plot([0 timevect(end)],[max(A) max(A)],'--','LineWidth',1.25)
settlingtime = timevect(max(find(abs(A(1:45*60)-SP)>(SP*0.02))));
plot([settlingtime settlingtime],[10 75],'--','LineWidth',1.25)
settlingtimetext = sprintf('Settling time: %.1f min',settlingtime);
legend('Temperature','SP=55\circC',risetimetext,overshoottext,settlingtimetext,'location','southeast')
grid on
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
ylim([10 60])
title('Pessen Integral Rule PID')
yticks([0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75])
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('Pessen_55','-dpng','-r900')
close all

%%
A = load("Pessen_55_4l_sous_vide.txt");
SP=55;
timevect = 0:1:length(A)-1;
timevect=timevect./60;
plot(timevect,A)
hold on
plot([0 timevect(end)],[SP-1 SP-1],'--','LineWidth',1.25)
plot([0 timevect(end)],[SP SP],'--','LineWidth',1.25)
plot([0 timevect(end)],[SP+1 SP+1],'--','LineWidth',1.25)
plot([66 66],[10 60],'--','LineWidth',1.25)
plot([284.7 284.7],[10 60],'--','LineWidth',1.25)
plot([464.3 464.3],[10 60],'--','LineWidth',1.25)
plot([556.1 556.1],[10 60],'--','LineWidth',1.25)
grid on
legend('Temperature','54\circC','SP=55\circC','56\circC','Cold water added','Meat added','Opening lid and moving stuff around','Bearnaise added','location','southeast')
set(gcf,'color','w');
xlabel('Time [min]')
ylabel('Temperature [\circC]')
title('Pessen Integral Rule PID')
%%
set(gcf,'color','w');
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 16 9])
print('Pessen_55_sous_vide','-dpng','-r900')
close all