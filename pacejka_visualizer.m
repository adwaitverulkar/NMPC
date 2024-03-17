clc, clear, close all;
f = figure;
set(gcf, 'Position', get(0, 'Screensize'));
ax = axes('Parent',f,'position',[0.13 0.39  0.77 0.54]);

B = 10;
C = 1.6;
D = 1.2;
E = 0.1;

h = fplot(@(slip) pacejka(1, B, C, D, E, slip), [0 1]); 




b = uicontrol('Parent',f,'Style','slider','Position',[1500,400,1000,23],...
              'value', B, 'min', 5, 'max', 20);

c = uicontrol('Parent',f,'Style','slider','Position',[1500,300,1000,23],...
              'value', C, 'min', 1.2, 'max', 1.6);

d = uicontrol('Parent',f,'Style','slider','Position',[1500,200,1000,23],...
              'value', D, 'min', 0.5, 'max', 2);

e = uicontrol('Parent',f,'Style','slider','Position',[1500,100,1000,23],...
              'value', E, 'min', 0.1, 'max', 0.5);

bgcolor = f.Color;

bl1 = uicontrol('Parent',f,'Style','text','Position',[1440,400,23,23],...
                'String','5', 'fontsize', 15);
bl2 = uicontrol('Parent',f,'Style','text','Position',[2520,400,56,23],...
                'String','20', 'fontsize', 15);
bl3 = uicontrol('Parent',f,'Style','text','Position',[2000,350,100,23],...
                'String','B', 'fontsize', 15);

cl1 = uicontrol('Parent',f,'Style','text','Position',[1440,300,56,23],...
                'String','1.2', 'fontsize', 15);
cl2 = uicontrol('Parent',f,'Style','text','Position',[2520,300,56,23],...
                'String','1.6', 'fontsize', 15);
cl3 = uicontrol('Parent',f,'Style','text','Position',[2000,250,100,23],...
                'String','C', 'fontsize', 15);


dl1 = uicontrol('Parent',f,'Style','text','Position',[1440,200,56,23],...
                'String','0.5', 'fontsize', 15);
dl2 = uicontrol('Parent',f,'Style','text','Position',[2520,200,56,23],...
                'String','2.0', 'fontsize', 15);
dl3 = uicontrol('Parent',f,'Style','text','Position',[2000,150,100,23],...
                'String','D', 'fontsize', 15);

el1 = uicontrol('Parent',f,'Style','text','Position',[1440,100,56,23],...
                'String','0.1', 'fontsize', 15);
el2 = uicontrol('Parent',f,'Style','text','Position',[2520,100,56,23],...
                'String','0.5', 'fontsize', 15);
el3 = uicontrol('Parent',f,'Style','text','Position',[2000,50,100,23],...
                'String','E', 'fontsize', 15);

b.Callback = @(es, ed) fplot(@(slip) pacejka(1, es.Value, C, D, E, slip), [0 1]);
c.Callback = @(es, ed) fplot(@(slip) pacejka(1, B, es.Value, D, E, slip), [0 1]);
d.Callback = @(es, ed) fplot(@(slip) pacejka(1, B, C, es.Value, E, slip), [0 1]);
e.Callback = @(es, ed) fplot(@(slip) pacejka(1, B, C, D, es.Value, slip), [0 1]);


