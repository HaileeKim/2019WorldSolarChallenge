clear all

delete(instrfindall);

s = serial('COM3');
set(s,'BaudRate',38400,'Timeout',10);
fopen(s);

xlsx_count = readmatrix('Comeon.xlsx','Range','H1');
xlsx_count = xlsx_count(1);

cols = [ "Time","Current1","Current2","Voltage","Speed","SOC","Distance"];
writematrix(cols,'Comeon.xlsx','Range','A1')

formatOut = 'dd/hh:MM:ss';
xlsx_write_list = [];

time = 0;
data1 = 0;
data2 = 0;
data3 = 0;
data4 = 0;
data5 = 0;
data6 = 0;

count = 0;
scrollWidth = 10;
present_value_number = 0;
tictoc_value = 0.1;
error_count = 0;

plot_title_size = 15;
plot_ylabel_size = 10;
plot_test_size = 25;

%Current Plot[Motor]
subplot 231
plotGraph_Motor = plot(time,data1);    
title('Current[Motor]','FontSize',plot_title_size);
ylabel('Ampere[A]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 50]);
text1 = text(0,0,'0','fontsize',1);


%Current Plot[MPPT]
subplot 232
plotGraph_MPPT = plot(time,data2);    
title('Current[MPPT]','FontSize',plot_title_size);
ylabel('Ampere[A]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 10]);
text2 = text(0,0,'0','fontsize',1);

%Current Plot[MPPT]
subplot 233
plotGraph_Voltage = plot(time,data3);    
title('Voltage','FontSize',plot_title_size);
ylabel('Speed[km/h]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 130]);
text3 = text(0,0,'0','fontsize',1);

%Current Plot[SPEED]
subplot 234
plotGraph_SPEED = plot(time,data4);    
title('Speed','FontSize',plot_title_size);
ylabel('Speed[km/h]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 115]);
text4 = text(0,0,'0','fontsize',1);

%Current Plot[SOC]
subplot 235
plotGraph_SOC = plot(time,data5);    
title('SOC','FontSize',plot_title_size);
ylabel('SOC[Ah]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 50]);
text5 = text(0,0,'0','fontsize',1);

%Current Plot[Distance]
subplot 236
plotGraph_Distance = plot(time,data6);    
title('Distance','FontSize',plot_title_size);
ylabel('Distance[km]','FontSize',plot_ylabel_size);
grid on;axis([0 10 -1.5 650]);
text6 = text(0,0,'0','fontsize',1);

figure(1)
pause(3)
tic
while 1
    while present_value_number < 160 
        try
            serial_data = fscanf(s);

            data_string_list = split(serial_data,' ');

            data_list = [str2double(data_string_list{1}) str2double(data_string_list{2}) str2double(data_string_list{3}) str2double(data_string_list{4}) str2double(data_string_list{5}) str2double(data_string_list{6})];
            count = count + 1;    
            time(count) = toc*tictoc_value;

            data1(count) = data_list(1);      
            data2(count) = data_list(2);
            data3(count) = data_list(3);
            data4(count) = data_list(4);
            data5(count) = data_list(5);
            data6(count) = data_list(6);

            subplot 231
            delete(text1)
            set(plotGraph_Motor,'XData',time(time > time(count)-scrollWidth),'YData',data1(time > time(count)-scrollWidth))
            axis([time(count)-scrollWidth+2 time(count) -1.5 50]);

            text1 = text(time(count),1,num2str(data1(count)),'fontsize',plot_test_size);
            text1;

            subplot 232
            delete(text2)
            set(plotGraph_MPPT,'XData',time(time > time(count)-scrollWidth),'YData',data2(time > time(count)-scrollWidth))
            axis([time(count)-scrollWidth+2 time(count) -0.25 10]);
            text2 = text(time(count),0.3,num2str(data2(count)),'fontsize',plot_test_size);
            text2;

            subplot 233
            delete(text3)
            set(plotGraph_Voltage,'XData',time(time > time(count)-scrollWidth),'YData',data4(time > time(count)-scrollWidth))
            axis([time(count)-scrollWidth+2 time(count) -1.5 130]);
            text3 = text(time(count),6,num2str(data3(count)),'fontsize',plot_test_size);
            text3;

            subplot 234
            delete(text4)
            set(plotGraph_SPEED,'XData',time(time > time(count)-scrollWidth),'YData',data4(time > time(count)-scrollWidth));
            axis([time(count)-scrollWidth+2 time(count) -1.5 115]);
            text4 = text(time(count),5,num2str(data4(count)),'fontsize',plot_test_size);
            text4;
            
            subplot 235
            delete(text5)
            set(plotGraph_SOC,'XData',time(time > time(count)-scrollWidth),'YData',data5(time > time(count)-scrollWidth));
            axis([time(count)-scrollWidth+2 time(count) -1.5 50]);
            text5 = text(time(count),1,num2str(data5(count)),'fontsize',plot_test_size);
            text5;
            
            subplot 236
            delete(text6)
            set(plotGraph_Distance,'XData',time(time > time(count)-scrollWidth),'YData',data6(time > time(count)-scrollWidth));
            axis([time(count)-scrollWidth+2 time(count) -1.5 650]);
            text6 = text(time(count),30,num2str(data6(count)),'fontsize',plot_test_size);
            text6;

            present_value_number = present_value_number + 1;
            
            time_xlsx = datestr(now,formatOut);
            %xlsx_write_list = [data_list(1,:),convertCharsToStrings(time_xlsx),xlsx_count+1];
            writematrix([convertCharsToStrings(time_xlsx),data_list(1,:),xlsx_count+1],'Comeon.xlsx','Range',strcat('A',num2str(xlsx_count)));
            xlsx_count = xlsx_count + 1;
            
            pause(0.01);
        catch expception
            pause(0.1)
            error_count = error_count + 1
            continue
        end
    end
    try
        serial_data = fscanf(s);

        data_string_list = split(serial_data,' ');

        data_list = [str2double(data_string_list{1}) str2double(data_string_list{2}) str2double(data_string_list{3}) str2double(data_string_list{4}) str2double(data_string_list{5}) str2double(data_string_list{6})];
        count = present_value_number+1;  
        time(count) = toc*tictoc_value;

        data1(count) = data_list(1);      
        data2(count) = data_list(2);
        data3(count) = data_list(3);
        data4(count) = data_list(4);
        data5(count) = data_list(5);
        data6(count) = data_list(6);

        subplot 231
        delete(text1)
        set(plotGraph_Motor,'XData',time(time > time(count)-scrollWidth),'YData',data1(time > time(count)-scrollWidth))
        axis([time(count)-scrollWidth+2 time(count) -1.5 50]);
        text1 = text(time(count),1,num2str(data1(count)),'fontsize',plot_test_size);
        text1;

        subplot 232
        delete(text2)
        set(plotGraph_MPPT,'XData',time(time > time(count)-scrollWidth),'YData',data2(time > time(count)-scrollWidth))
        axis([time(count)-scrollWidth+2 time(count) -0.25 10]);
        text2 = text(time(count),0.3,num2str(data2(count)),'fontsize',plot_test_size);
        text2;

        subplot 233
        delete(text3)
        set(plotGraph_Voltage,'XData',time(time > time(count)-scrollWidth),'YData',data4(time > time(count)-scrollWidth))
        axis([time(count)-scrollWidth+2 time(count) -1.5 130]);
        text3 = text(time(count),6,num2str(data3(count)),'fontsize',plot_test_size);
        text3;

        subplot 234
        delete(text4)
        set(plotGraph_SPEED,'XData',time(time > time(count)-scrollWidth),'YData',data4(time > time(count)-scrollWidth));
        axis([time(count)-scrollWidth+2 time(count) -1.5 115]);
        text4 = text(time(count),5,num2str(data4(count)),'fontsize',plot_test_size);
        text4;
        
        subplot 235
        delete(text5)
        set(plotGraph_SOC,'XData',time(time > time(count)-scrollWidth),'YData',data5(time > time(count)-scrollWidth));
        axis([time(count)-scrollWidth+2 time(count) -1.5 50]);
        text5 = text(time(count),1,num2str(data5(count)),'fontsize',plot_test_size);
        text5;

        subplot 236
        delete(text6)
        set(plotGraph_Distance,'XData',time(time > time(count)-scrollWidth),'YData',data6(time > time(count)-scrollWidth));
        axis([time(count)-scrollWidth+2 time(count) -1.5 650]);
        text6 = text(time(count),30,num2str(data6(count)),'fontsize',plot_test_size);
        text6;

        time_xlsx = datestr(now,formatOut);
        %xlsx_write_list = [data_list(1,:),convertCharsToStrings(time_xlsx),xlsx_count+1];
        writematrix([convertCharsToStrings(time_xlsx),data_list(1,:),xlsx_count+1],'Comeon.xlsx','Range',strcat('A',num2str(xlsx_count)));
        xlsx_count = xlsx_count + 1;
        

        data1(1) = [];
        data2(1) = [];
        data3(1) = [];
        data4(1) = [];
        data5(1) = [];
        data6(1) = [];
        time(1) = [];
        pause(0.01);
    catch expception
            pause(0.1)
            error_count = error_count + 1
            continue
    end
    
end

fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;
 
 
disp('Session Terminated...');