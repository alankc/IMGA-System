%Data
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Experiment 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data = importdata('exp1_result_15t_5r.txt','\t',1);

x = data.data(:,1);
y =data.data(:,2);
z = data.data(:,3);
figure;
htm = heatmap(5:5:30,5:5:30,reshape(z,6,6));
set(gca,'fontsize',12)
set(findobj(htm,'type','text'),'fontsize',12); 
xlabel('Taxa de Elitismo (%)')
ylabel('Taxa de Mutação (%)')
title('Tempo (ms)')
%saveas(htm, 'images_pt_BR\exp1_time.png')

z = data.data(:,5);
figure;
htm = heatmap(5:5:30,5:5:30,reshape(z,6,6)*100);
set(gca,'fontsize',12)
set(findobj(htm,'type','text'),'fontsize',12); 
xlabel('Taxa de Elitismo (%)')
ylabel('Taxa de Mutação (%)')
title('Taxa de Sucesso (%)')
%saveas(htm, 'images_pt_BR\exp1_success.png')

% figure;
% islands = sort(sum(data.data(:,6:end)),'descend') * 1000;
% is = [sum(islands(1,1:5)) sum(islands(1,32:36)) sum(islands(1,6:31))];
% labels = {'Top 5','Minor 5', 'Others'};
% p = pie(is);
% set(gca,'fontsize',12)
% set(findobj(p,'type','text'),'fontsize',12); 
% title('Islands participation')
% legend(labels)
% colormap([28, 116, 157;
%           235, 231, 224;
%           198, 212, 225]*(1/255))
% saveas(gcf, 'images_pt_BR\exp1_participation.png')   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Experiment 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data = importdata('exp2_result_15t_5r.txt','\t',1);

x = data.data(:,1);
y =data.data(:,2);
z = data.data(:,3);
figure;
htm = heatmap(5:5:30,10:10:100,reshape(z,10,6));
set(gca,'fontsize',12)
set(findobj(htm,'type','text'),'fontsize',12); 
xlabel('Taxa de Migração (%)')
ylabel('Sub-iterações')
title('Tempo (ms)')
%saveas(htm, 'images_pt_BR\exp2_time.png')

z = data.data(:,5);
figure;
htm = heatmap(5:5:30,10:10:100,reshape(z,10,6)*100);
set(gca,'fontsize',12)
set(findobj(htm,'type','text'),'fontsize',12); 
xlabel('Taxa de Migração (%)')
ylabel('Sub-iterações')
title('Taxa de Sucesso (%)')
%saveas(htm, 'images_pt_BR\exp2_success.png')

% figure;
% islands = sort(sum(data.data(:,6:end)),'descend') * 1000;
% is = [sum(islands(1,1:5)) sum(islands(1,32:36)) sum(islands(1,6:31))];
% labels = {'Top 5','Minor 5', 'Others'};
% p = pie(is);
% set(gca,'fontsize',12)
% set(findobj(p,'type','text'),'fontsize',12); 
% title('Islands participation')
% legend(labels)
% colormap([28, 116, 157;
%           235, 231, 224;
%           198, 212, 225]*(1/255))
% saveas(gcf, 'images_pt_BR\exp2_participation.png')      