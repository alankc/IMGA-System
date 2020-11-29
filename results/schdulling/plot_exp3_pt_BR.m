%Data
clear all;

pos = ["Iteration","timeToScheduling","scheduledPercentage","fitnessExpected","fitnessObtained","energyExpected","energyObtained","timeExpected","timeObtained","payloadExpected","payloadObtained","meanNormEnergyE","stdNormEnergyE","meanNormEnergyO","stdNormEnergyO","meanNormPayloadE","stdNormPayloadE","meanNormPayloadO","stdNormPayloadO","-","Islands"];
% d5t = importdata('exp3_5t.txt','\t',2);
% d10t = importdata('exp3_10t.txt','\t',2);
% d15t = importdata('exp3_15t.txt','\t',2);
% d20t = importdata('exp3_20t.txt','\t',2);
% d25t = importdata('exp3_25t.txt','\t',2);
% d30t = importdata('exp3_30t.txt','\t',2);
% d35t = importdata('exp3_35t.txt','\t',2);
% d40t = importdata('exp3_40t.txt','\t',2);
% d50t = importdata('exp3_50t.txt','\t',2);

xdata = categorical({'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});
xdata = reordercats(xdata,{'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});
j = 1;
for i = 5:5:50
    filename = strcat(strcat('exp3_',string(i)), 't.txt');
    if isfile(filename)
        d(j) = importdata(filename,'\t',2);    
        j = j + 1;
    end
end


%All tasks scheduled
avg = zeros(1,9);
for i = 1:1:size(d,2)
   avg(i) = size(find(d(i).data(:,3) == 1),1)/10;
end
figure;
b = bar(xdata,avg);
b.FaceColor = [28, 116, 157]*(1/255);
set(gca,'fontsize',12)
set(findobj(b,'type','text'),'fontsize',12);
xlabel('Tarefas-Robôs')
ylabel('Porcentagem')
ylim([0 110])
title('Percentual de soluções completas')
text(1:length(avg),avg,num2str(avg'),'vert','bottom','horiz','center','FontSize',12); 
%saveas(gca, 'images_pt_BR\exp3_all_tasks.png')


% percent of Tasks scheduled
avg = zeros(1,9);
for i = 1:1:size(d,2)
   avg(i) = mean(d(i).data(:,3))*100;
end
figure;
b = bar(xdata,avg);
b.FaceColor = [28, 116, 157]*(1/255);
set(gca,'fontsize',12)
set(findobj(b,'type','text'),'fontsize',12);
xlabel('Tarefas-Robôs')
ylabel('Porcentagem')
ylim([0 110])
title('Percentual médio de tarefas alocadas')
text(1:length(avg),avg,num2str(round(avg,2)'),'vert','bottom','horiz','center','FontSize',12); 
%saveas(gca, 'images_pt_BR\exp3_allocated_tasks.png')


%Mean time to schedule
avg = zeros(1,9);
std_dev = zeros(1,9);
for i = 1:1:size(d,2)
   avg(i) = mean(d(i).data(:,2))/1000;
   std_dev(i) = std(d(i).data(:,2)/1000);
end
figure;
b = bar(xdata,avg);
b.FaceColor = [28, 116, 157]*(1/255);
hold on;
er = errorbar(xdata,avg,std_dev);
er.Color = [198, 212, 225]*(1/255);                            
er.LineStyle = 'none';  
er.LineWidth = 2.5;  
set(gca,'fontsize',12)
set(findobj(b,'type','text'),'fontsize',12);
xlabel('Tarefas-Robôs')
ylabel('Tempo (s)')
ylim([-8 100])
title('Tempo médio para alocar tarefas')
text(1:length(avg),avg,num2str(round(avg,2)'),'vert','bottom','horiz','center','FontSize',12); 
%saveas(gca, 'images_pt_BR\exp3_time.png')


%Islands participation
islands = zeros(1,36);
for i = 3:1:size(d,2)
   islands = islands + sum(d(i).data(:,21:end));
end
figure;
islandsO = islands;
islands = sort(islands,'descend');
is = [sum(islands(1,1:5)) sum(islands(1,32:36)) sum(islands(1,6:31))];
labels = {'Top 5','Minor 5', 'Others'};
p = pie(is);
set(gca,'fontsize',12)
set(findobj(p,'type','text'),'fontsize',12); 
title('Participação das Ilhas')
legend(labels)
colormap([28, 116, 157;
          235, 231, 224;
          198, 212, 225]*(1/255))
%saveas(gca, 'images_pt_BR\exp3_participation.png')      

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Table %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
coluna = 0;
avg_erro = 0;
avg_goal = 0;
std_goal = 0;
avg_obta = 0;
std_obta = 0;
%Fitness
coluna = coluna + 1;
for i = 1:1:size(d,2)
   lines = find(d(i).data(:,3) == 1); 
   goal = d(i).data(lines,4);
   obta = d(i).data(lines,5);
   
   avg_erro(i,coluna) = 100 * mean(obta./goal-1);
   
   avg_goal(i,coluna) = mean(goal);
   avg_obta(i,coluna) = mean(obta);
   
   std_goal(i,coluna) = std(goal);
   std_obta(i,coluna) = std(obta);
end

%Tempo Total
coluna = coluna + 1;
for i = 1:1:size(d,2)
   lines = find(d(i).data(:,3) == 1); 
   goal = d(i).data(lines,8);
   obta = d(i).data(lines,9);
   
   avg_erro(i,coluna) = 100 * mean(obta./goal-1);
   
   avg_goal(i,coluna) = mean(goal);
   avg_obta(i,coluna) = mean(obta);
   
   std_goal(i,coluna) = std(goal);
   std_obta(i,coluna) = std(obta);
end

%Energia Total
coluna = coluna + 1;
for i = 1:1:size(d,2)
   lines = find(d(i).data(:,3) == 1); 
   goal = d(i).data(lines,6);
   obta = d(i).data(lines,7);
   
   avg_erro(i,coluna) = 100 * mean(obta./goal-1);
   
   avg_goal(i,coluna) = mean(goal);
   avg_obta(i,coluna) = mean(obta);
   
   std_goal(i,coluna) = std(goal);
   std_obta(i,coluna) = std(obta);
end

%Energia Normalizada
coluna = coluna + 1;
for i = 1:1:size(d,2)
   lines = find(d(i).data(:,3) == 1); 
   goal = 100*d(i).data(lines,12);
   obta = 100*d(i).data(lines,14);
   
   avg_erro(i,coluna) = 100 * mean(obta./goal-1);
   
   avg_goal(i,coluna) = mean(goal);
   avg_obta(i,coluna) = mean(obta);
   
   std_goal(i,coluna) = std(goal);
   std_obta(i,coluna) = std(obta);
end

%STD da Energia Normalizada
% coluna = coluna + 1;
% for i = 1:1:size(d,2)
%    lines = find(d(i).data(:,3) == 1); 
%    goal = d(i).data(lines,13);
%    obta = d(i).data(lines,15);
%    
%    avg_erro(i,coluna) = 100 * mean(obta./goal-1);
%    
%    avg_goal(i,coluna) = mean(goal);
%    avg_obta(i,coluna) = mean(obta);
%    
%    std_goal(i,coluna) = std(goal);
%    std_obta(i,coluna) = std(obta);
% end

%Payload Normalizado
coluna = coluna + 1;
for i = 1:1:size(d,2)
   lines = find(d(i).data(:,3) == 1); 
   goal = 100*d(i).data(lines,16);
   obta = 100*d(i).data(lines,18);
   
   avg_erro(i,coluna) = 100 * mean(obta./goal-1);
   
   avg_goal(i,coluna) = mean(goal);
   avg_obta(i,coluna) = mean(obta);
   
   std_goal(i,coluna) = std(goal);
   std_obta(i,coluna) = std(obta);
end

%STD do Payload Normalizado
% coluna = coluna + 1;
% for i = 1:1:size(d,2)
%    lines = find(d(i).data(:,3) == 1); 
%    goal = d(i).data(lines,17);
%    obta = d(i).data(lines,19);
%    
%    avg_erro(i,coluna) = 100 * mean(obta./goal-1);
%    
%    avg_goal(i,coluna) = mean(goal);
%    avg_obta(i,coluna) = mean(obta);
%    
%    std_goal(i,coluna) = std(goal);
%    std_obta(i,coluna) = std(obta);
% end

avg_erro_R = round(avg_erro,2);


% avg_goal = round(avg_goal, 4);
% std_goal = round(std_goal, 4);
% avg_obta = round(avg_obta, 4);
% std_obta = round(std_obta, 4);

