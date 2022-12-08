clear all 
close all
t05  = [15.9, 16.0, 16.7];
t04  = [16.7, 16.0, 16.5];
t03  = [14.5, 15.0, 14.8];
t02  = [13.8, 13.8, 13.9];
t01  = [14.0, 14.0, 14.0];
t00 = [14.0, 14.3, 14.2];
t_01 = [14.0, 15.0, 14.0];
t_02 = [13.9, 14.5, 14.5];
NoSwing = [13.5, 13.4, 13.5];

t05_mean  = mean(t05);
t04_mean  = mean(t04);
t03_mean  = mean(t03);
t02_mean  = mean(t02);
t01_mean  = mean(t01);
t00_mean= mean(t00);
t_01_mean = mean(t_01);
t_02_mean = mean(t_02);
NoSwing_mean = mean(NoSwing);

t05_std  = std(t05);
t04_std  = std(t04);
t03_std  = std(t03);
t02_std  = std(t02);
t01_std  = std(t01);
t00_std= std(t00);
t_01_std = std(t_01);
t_02_std= std(t_02);
NoSwing_std = std(NoSwing);


std_arr = [t05_std, t04_std, t03_std, t02_std, t01_std,t00_std,t_01_std,t_02_std];
arr = [t05_mean, t04_mean, t03_mean, t02_mean, t01_mean,t00_mean,t_01_mean,t_02_mean]; %, NoSwing];
x = [-0.05, -0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02];

plot(x, arr, 'b.', "MarkerSize", 17)
hold on
plot(0, NoSwing_mean, 'r.', "MarkerSize", 17)
errorbar(x, arr,std_arr, std_arr, 'b.', "MarkerSize", 17)
errorbar(0,NoSwing_mean,NoSwing_std, NoSwing_std, 'r.', "MarkerSize", 17)
grid on
legend("With arm swing", "No arm swing")
xlabel("Time Delay [s]")
ylabel("Jump Height [cm]")

pincrease_1 = (t04_mean - NoSwing_mean)/ NoSwing_mean;
pincrease_2 =(t04_mean - t02_mean)/ t02_mean;
