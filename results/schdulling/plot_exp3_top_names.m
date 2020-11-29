clear all;
xdata = categorical({'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});
xdata = reordercats(xdata,{'5-3';'10-4';'15-5';'20-6';'25-7';'30-9';'35-10';'40-12';'50-15'});

j = 1;
for i = 5:5:50
    filename = strcat(strcat('EXP3\MY\exp3_',string(i)), 't.txt');
    if isfile(filename)
        d(j) = importdata(filename,'\t',2);    
        j = j + 1;
    end
end

islands = zeros(1,36);
for i = 1:1:size(d,2)
   islands = islands + sum(d(i).data(:,21:end));
end
figure;
islandsO = islands;
islands = sort(islands,'descend');
is = [sum(islands(1,1:9)) sum(islands(1,32:36)) sum(islands(1,10:31))];
labels = {'Top 5','Minor 5', 'Others'};
p = pie(is);
set(gca,'fontsize',12)
set(findobj(p,'type','text'),'fontsize',12); 
title('Islands participation')
legend(labels)
colormap([28, 116, 157;
          235, 231, 224;
          198, 212, 225]*(1/255))
      
% selct = ["rouletteWheel" "rank" "tournament"];
% cross = ["order" "orderBased" "positionBased"];
% mutat = ["exchange" "displacement" "inversion" "scramble"];
selct = ["RWS" "RS" "TS"];
cross = ["OX" "OBX" "PX"];
mutat = ["EM" "DM" "IM" "SM"];

names(1,1) = "";
names(2,1) = "";
l = 1;
tst = ismember(islandsO,islands(1,1:9));

for i = 1:1:3
   for j = 1:1:3
       for k = 1:1:4         
        names(1,l) = strcat(selct(i),"-",cross(j),"-", mutat(k));
        if (tst(l))
            names(2,l) = strcat("---",string(islandsO(l)),"---");
        else
            names(2,l) = string(islandsO(l));
        end
        l = l + 1;
       end    
   end 
end

figure;
xdata = categorical(names(1,:));
xdata = reordercats(xdata,names(1,:));
b = bar(xdata,islandsO);
b.FaceColor = [28, 116, 157]*(1/255);
title('Number of solutions found by each island')
ylabel('Number of Solutions')
xlabel('Selection-Crossover-Mutation')
set(gca,'fontsize',36)
yticks(0:200:1700)
ylim([0 1700])
grid on;
% ["rank-orderBased-exchange";"---1410---"]
% ["rank-positionBased-exchange";"---1382---"]
% ["tournament-order-exchange";"---1368---"]
% ["tournament-orderBased-exchange";"---1597---"]
% ["tournament-positionBased-exchange";"---1651---"]

% ["rank-order-exchange";"---304---"]
% ["rank-orderBased-exchange";"---378---"]
% ["rank-positionBased-exchange";"---331---"]
% ["tournament-orderBased-exchange";"---295---"]
% ["tournament-positionBased-exchange";"---320---"]
