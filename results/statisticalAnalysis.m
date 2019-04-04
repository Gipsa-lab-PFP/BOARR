%% read log text file
clear all
close all

D = readtable('FullBenchmarkResults.txt','Delimiter',' ');

%% work on positive tests 
close all

DOK=D(D.Var1 > 0,:);
testNumber = size(D,1);
sucessNumber = size(DOK,1);
fprintf('Probability of Sucess : %d (%d/%d) \n',sucessNumber/testNumber,sucessNumber,testNumber);

figure()
edges = [min(DOK.Var3)-1:1:max(DOK.Var3)+1]; 
H = histogram(DOK.Var3,edges);
title('Sucessfull tests : Histogram of the Travelled Distance')
fprintf('Secondary Indicators Format : "Name : Mean [First Decile, Ninth Decile]"\n')
fprintf('Travelled Distance (m) : %d [%d, %d] \n',quantile(DOK.Var3,0.5),quantile(DOK.Var3,0.1),quantile(DOK.Var3,0.9));


figure()
edges = [min(DOK.Var4)-1:0.5:max(DOK.Var4)+1]; 
H = histogram(DOK.Var4,edges);
title('Sucessfull tests : Histogram of the Completion Time')
fprintf('Time to complete the tet (s) : %d [%d, %d] \n',quantile(DOK.Var4,0.5),quantile(DOK.Var4,0.1),quantile(DOK.Var4,0.9));

figure()
edges = [min(DOK.Var5)-1:0.5:max(DOK.Var5)+1]; 
H = histogram(DOK.Var5,edges);
title('Sucessfull tests : Histogram of the Consumed Energy')
fprintf('Consumed Energy (Wh) : %d [%d, %d] \n',quantile(DOK.Var5,0.5),quantile(DOK.Var5,0.1),quantile(DOK.Var5,0.9));

figure()
averageSpeed = DOK.Var3./DOK.Var4;
edges = [min(averageSpeed)-0.1:0.1:max(averageSpeed)+.1]; 
H = histogram(averageSpeed,edges);
title('Sucessfull tests : Histogram of the flying Speed')
fprintf('Average Speed (m/s) : %d [%d, %d] \n',quantile(averageSpeed,0.5),quantile(averageSpeed,0.1),quantile(averageSpeed,0.9));


%% work on non finished tests
DFAILED = D(D.Var1 < 1,:);

%Print number of tests which are not finished after 60s, some of them are
%bugged, I want to erase those. 
failNumber = size(DFAILED,1);


figure()
edges = [min(DFAILED.Var4)-1:1:max(DFAILED.Var4)+1]; 
H = histogram(DFAILED.Var4,edges);
title('Failed test : Histogram of the Flight Length Before Collision')
fprintf('Failed tests : Flight Length Before Collision (s) : %d [%d, %d] \n',quantile(DFAILED.Var4,0.5),quantile(DFAILED.Var4,0.1),quantile(DFAILED.Var4,0.9));

figure()
edges = [min(DFAILED.Var6)-1:1:max(DFAILED.Var6)+1]; 
H = histogram(DFAILED.Var6,edges);
title('Failed test : Histogram of the Linear Distance Before Collision')
fprintf('Failed tests : Linear Distance Before Collision (m) : %d [%d, %d] \n',quantile(DFAILED.Var6,0.5),quantile(DFAILED.Var6,0.1),quantile(DFAILED.Var6,0.9));

%% save the table or get info on the table
% FileName=[datestr(now, 'yyyy-mm-dd_HH:MM:SS'),'_benchmarkAnalysis.dat'];
% save(FileName,'D') ;
 

