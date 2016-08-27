%% Figure Genreation
figr  = open('Simulation_Pane.fig');
set(figr,'Name','AMCL Localization');

hold(figr.Children(3),'on');
hold(figr.Children(3),'on');
%% Map Borders Generation & Internal Structure Generation

line(figr.Children(3),[0 0],[0 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),[0 50],[0 0],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),[50 50],[0 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),[50 0],[50 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),'XData',[5 10 25 40 45 40 45 20 15 10],'YData',[5 10 20 15 20 35 45 45 35 40],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),'XData',[5 15 5 5],'YData',[20 30 35 45],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),'XData',[25 30 25 20],'YData',[25 30 40 30],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(3),'XData',[15 15 25 35],'YData',[0 5 10 5],'LineWidth',3,'Color',[1 0 0])
xlim(figr.Children(3),[-2 52])
ylim(figr.Children(3),[-2 52])
grid(figr.Children(3),'on')
grid(figr.Children(3),'minor')

line(figr.Children(4),[0 0],[0 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),[0 50],[0 0],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),[50 50],[0 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),[50 0],[50 50],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),'XData',[5 10 25 40 45 40 45 20 15 10],'YData',[5 10 20 15 20 35 45 45 35 40],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),'XData',[5 15 5 5],'YData',[20 30 35 45],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),'XData',[25 30 25 20],'YData',[25 30 40 30],'LineWidth',3,'Color',[1 0 0])
line(figr.Children(4),'XData',[15 15 25 35],'YData',[0 5 10 5],'LineWidth',3,'Color',[1 0 0])

xlim(figr.Children(4),[-2 52])
ylim(figr.Children(4),[-2 52])
grid(figr.Children(4),'on')
grid(figr.Children(4),'minor')

%% Initialization 

CATRobot = CATBot;
CATRobot.spawn_CATBot(figr.Children(3).Children,figr.Children(3),figr.Children(4))
CATRobot.get_sensor_data()
CATRobot.monte_carlo_localization([0,0])
CATRobot.plot_Particles(figr.Children(4))
% CATRobot.plot_rays(figr.Children(3))