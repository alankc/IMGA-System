%Data
clear all;

pos = ["Iteration","timeToScheduling","scheduledPercentage","fitnessExpected","fitnessObtained","energyExpected","energyObtained","timeExpected","timeObtained","payloadExpected","payloadObtained","meanNormEnergyE","stdNormEnergyE","meanNormEnergyO","stdNormEnergyO","meanNormPayloadE","stdNormPayloadE","meanNormPayloadO","stdNormPayloadO","-","Islands"];

xdata = categorical({'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});
xdata = reordercats(xdata,{'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});

j_my = 1;
j_9islnds = 1;
j_gapure = 1;

for i = 5:5:50
    filename = strcat(strcat('EXP3/MY/exp3_',string(i)), 't.txt');
    if isfile(filename)
        d_my(j_my) = importdata(filename,'\t',2);    
        j_my = j_my + 1;
    end
    
    filename = strcat(strcat('EXP3/9_ISLANDS/exp3_',string(i)), 't.txt');
    if isfile(filename)
        d_9islands(j_9islnds) = importdata(filename,'\t',2);    
        j_9islnds = j_9islnds + 1;
    end
    
    filename = strcat(strcat('EXP3/GA_PURE/exp3_',string(i)), 't.txt');
    if isfile(filename)
        d_gapure(j_gapure) = importdata(filename,'\t',2);    
        j_gapure = j_gapure + 1;
    end       
end

%All tasks scheduled
n_solution_my = zeros(1,9);
n_solution_9islands = zeros(1,9);
n_solution_gapure = zeros(1,9);
for i = 1:1:9
   n_solution_my(i) = size(find(d_my(i).data(:,3) == 1),1)/10;
   n_solution_9islands(i) = size(find(d_9islands(i).data(:,3) == 1),1)/10;
   n_solution_gapure(i) = size(find(d_gapure(i).data(:,3) == 1),1)/10;
end
figure('units','normalized','outerposition',[0 0 1 1]);
b = bar(xdata, [n_solution_my; n_solution_9islands; n_solution_gapure]');
text((1:length(n_solution_my))-0.23,n_solution_my,num2str(n_solution_my'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_9islands)),n_solution_9islands,num2str(n_solution_9islands'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_gapure))+0.23,n_solution_gapure,num2str(n_solution_gapure'),'vert','bottom','horiz','center','FontSize',14); 
set(gca,'fontsize',36)
set(findobj(b,'type','text'),'fontsize',36);
xlabel('Tarefas-Robôs')
ylabel('Porcentagem')
ylim([0 105])
title('Percentual de soluções completas')
legend({'36 Ilhas';'9 melhores ilhas';'Algoritmo Genético Puro'},'FontSize',36)
grid on;
saveas(gca, 'images_pt_BR\exp3_comp_all_tasks.png')

% percent of Tasks scheduled
n_solution_my = zeros(1,9);
n_solution_9islands = zeros(1,9);
n_solution_gapure = zeros(1,9);
for i = 1:1:9
    
   n_solution_my(i) = round(mean(d_my(i).data(:,3))*100,1);
   n_solution_9islands(i) = round(mean(d_9islands(i).data(:,3))*100,1);
   n_solution_gapure(i) = round(mean(d_gapure(i).data(:,3))*100,1);
end
figure('units','normalized','outerposition',[0 0 1 1]);
b = bar(xdata, [n_solution_my; n_solution_9islands; n_solution_gapure]');
text((1:length(n_solution_my))-0.23,n_solution_my,num2str(n_solution_my'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_9islands)),n_solution_9islands,num2str(n_solution_9islands'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_gapure))+0.23,n_solution_gapure,num2str(n_solution_gapure'),'vert','bottom','horiz','center','FontSize',14); 
set(gca,'fontsize',36)
set(findobj(b,'type','text'),'fontsize',36);
xlabel('Tarefas-Robôs')
ylabel('Porcentagem')
ylim([0 105])
title('Percentual médio de tarefas alocadas')
legend({'36 Ilhas';'9 melhores ilhas';'Algoritmo Genético Puro'},'FontSize',36,'Location','southeast')
grid on;
saveas(gca, 'images_pt_BR\exp3_comp_allocated_tasks.png')

% Mean time to schedule
n_solution_my = zeros(1,9);
n_solution_9islands = zeros(1,9);
n_solution_gapure = zeros(1,9);
for i = 1:1:9
    
   n_solution_my(i) = round(mean(d_my(i).data(:,2))/1000,1);
   n_solution_9islands(i) = round(mean(d_9islands(i).data(:,2))/1000,1);
   n_solution_gapure(i) = round(mean(d_gapure(i).data(:,2))/1000,1);
end
figure('units','normalized','outerposition',[0 0 1 1]);
b = bar(xdata, [n_solution_my; n_solution_9islands; n_solution_gapure]');
text((1:length(n_solution_my))-0.23,n_solution_my,num2str(n_solution_my'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_9islands)),n_solution_9islands,num2str(n_solution_9islands'),'vert','bottom','horiz','center','FontSize',14); 
text((1:length(n_solution_gapure))+0.23,n_solution_gapure,num2str(n_solution_gapure'),'vert','bottom','horiz','center','FontSize',14); 
set(gca,'fontsize',36)
set(findobj(b,'type','text'),'fontsize',36);
xlabel('Tarefas-Robôs')
ylabel('Tempo (s)')
ylim([0 100])
title('Tempo médio para alocar tarefas')
legend({'36 Ilhas';'9 melhores ilhas';'Algoritmo Genético Puro'},'FontSize',36,'Location','northwest')
grid on;
saveas(gca, 'images_pt_BR\exp3_comp_time.png')
