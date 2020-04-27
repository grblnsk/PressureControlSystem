% MIT License
% 
% Copyright (c) 2020 Jacek Garbulinski
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.


%% INITIALIZATION
% CLOSE SERIAL
if (exist('s'))
    fclose(s);
    delete(s);
    clear s;
end
%%
% OPEN SERIAL
clear
comPort = seriallist
% comPort='COM14';
%CREATE SERIAL OBJECT
comPort(1)
s = serial(comPort(1))
%get(s)
s.Timeout = 3;
s.InputBufferSize = 24*4*3;
%OPEN SERIAL
fopen(s);

rawSamples = zeros(1000000,1);
timestamps = zeros(1000000,1);
cmd.start = 'S';
cmd.stop = 'X';
cmd.request = 'R';
cmd.continuous = 'C';
cmd.demand = 'D';
cmd.input = 'I';

cmd.int = 'i';
cmd.dec = 'd';


%%VALVE-SENSOR PAIR DEPENDENT
cmd.valve1 = '1';
cmd.valve2 = '2';
cmd.valve3 = '3';

cmd.rctrl = 'J';
cmd.keep = 'K';

t_wait = 0.05;
t_plot = 0.5;
showMax = round(10/t_wait);

datafig = figure('Name','Measured Data', 'Position', [30 100 850 645]);
subplot(3,1,1)
x1 = zeros(2,1);
y1 = zeros(2,1);
p1 = plot(x1,y1);
ylabel('Pressure, psi')
p1.XDataSource = 'x1';
p1.YDataSource = 'y1';
t1_handle = title(['Controller 1 | Ramp Press:' num2str(0)...
    ' Set Press:' num2str(0)]);
grid minor

hold on

subplot(3,1,2)
x2 = zeros(2,1);
y2 = zeros(2,1);
p2 = plot(x2,y2);
ylabel('Pressure, psi')
p2.XDataSource = 'x2';
p2.YDataSource = 'y2';
t2_handle = title(['Controller 2 | Ramp Press:' num2str(0)...
    ' Set Press:' num2str(0)])
grid minor

hold on
subplot(3,1,3)
x3 = zeros(2,1);
y3 = zeros(2,1);
p3 = plot(x3,y3);
ylabel('Pressure, psi')
p3.XDataSource = 'x3';
p3.YDataSource = 'y3';
xlabel('Time(s)')
t3_handle = title(['Controller 3 | Ramp Press:' num2str(0)...
    ' Set Press:' num2str(0)])
grid minor

setGoalPressure(s,'1',0);
setGoalPressure(s,'2',0);
setGoalPressure(s,'3',0);


fig = uifigure('Position',[950 100 550 698]);
cg11 = uilabel(fig,'Position',[30+200 500 280 240], 'FontSize', 80, 'Text', '0.00');
cg21 = uilabel(fig,'Position',[30+200 500-1*250 280 240], 'FontSize',80, 'Text', '0.00');
cg31 = uilabel(fig,'Position',[30+200 500-2*250 280 240], 'FontSize',80, 'Text', '0.00');
controlpanel(fig);


fig2 = uifigure('Position',[950 100 550 698/3]);
cg4 = uilabel(fig2,'Position',[30 80 240 240], 'FontSize', 12, 'Text', 'Controller ALL: Set Pressure, psi');
cg41 = uilabel(fig2,'Position',[30 30 280 240], 'FontSize',80, 'Text', '0.00');
sld4 = uislider(fig2,...
    'Limits', [0 110],...
    'MajorTicks', 0:10:110,...
    'Position',[30 50 400 240],...
    'ValueChangedFcn',@(sld,event) updateGauge4(sld,cg41));

a2p1 = @(adc) adc;
a2p2 = @(adc) adc;
a2p3 = @(adc) adc;


hold off
%% MEASUREMENT
flushinput(s)

disp('SEND STOP TO TEST COMM. AND STOP PREVIOUS MEASUREMENT')

sendCommand(s,cmd.stop);

disp('SEND START')
sendCommand(s,cmd.start);
disp('START FREAD')

pause(0.1)
beep;

% A dialog to stop the loop
MessageBox = msgbox( 'Click OK to stop the measurement', 'Stop Meas.');
MessageBox.Position = [30 50 150 50]

k=1;
tic;
tp1 = toc;
while ishandle( MessageBox )
    t1 = toc;
    
    %disp('SEND Request')
    sendCommand(s, cmd.request);
    rawSamples(k,1) = fread(s,1,'uint32');
    %rawSamples(k,2:4) = fread(s,3,'float32');
    rawSamples(k,2:10) = fread(s,2*3+3,'float32');
    rawSamples(k,17) = toc;
    
    t2 = toc;
    while t2-t1<t_wait
        t2 = toc;
    end
    
    t1_handle.String = ['Controller 1 | Ramp Press:' num2str(rawSamples(k,6)) ' Set Press:' num2str(rawSamples(k,5))];
    t2_handle.String = ['Controller 2 | Ramp Press:' num2str(rawSamples(k,8)) ' Set Press:' num2str(rawSamples(k,7))];
    t3_handle.String = ['Controller 3 | Ramp Press:' num2str(rawSamples(k,10)) ' Set Press:' num2str(rawSamples(k,9))];
        
    if(k<=showMax)
        x1(k) = rawSamples(k,17);
        y1(k) = a2p1(rawSamples(k,2));
        x2(k) = rawSamples(k,17);
        y2(k) = a2p1(rawSamples(k,3));
        x3(k) = rawSamples(k,17);
        y3(k) = a2p1(rawSamples(k,4));
    else
        x1 = x1(2:showMax);
        y1 = y1(2:showMax);
        x2 = x2(2:showMax);
        y2 = y2(2:showMax);
        x3 = x3(2:showMax);
        y3 = y3(2:showMax);
        x1(showMax) = rawSamples(k,17);
        y1(showMax) = a2p1(rawSamples(k,2));
        x2(showMax) = rawSamples(k,17);
        y2(showMax) = a2p1(rawSamples(k,3));
        x3(showMax) = rawSamples(k,17);
        y3(showMax) = a2p1(rawSamples(k,4));
    end
    
    tp2 = toc;
    tp_temp = tp2-tp1;
    if (t_plot<tp_temp)
        tp1=toc;
        cg11.Text = sprintf('%.2f',round( rawSamples(k,2),2 ));
        cg21.Text = sprintf('%.2f',round( rawSamples(k,3),2 ));
        cg31.Text = sprintf('%.2f',round( rawSamples(k,4),2 ));
        
        refreshdata
        drawnow
    end
    k = k+1;
    
    
end
close(fig)
close(fig2)
close(datafig)
% end


function controlpanel(fig)
% Create figure window and components
cg1 = uilabel(fig,'Position',[30 560 240 240], 'FontSize', 12, 'Text', 'Controller 1: Set Pressure, psi');
cg2 = uilabel(fig,'Position',[30 560-1*250 240 240], 'FontSize',12, 'Text', 'Controller 2: Set Pressure, psi');
cg3 = uilabel(fig,'Position',[30 560-2*250 240 240], 'FontSize',12, 'Text', 'Controller 3: Set Pressure, psi');

cg1 = uilabel(fig,'Position',[230 560 240 240], 'FontSize', 12, 'Text', 'Controller 1: Meas. Pressure');
cg2 = uilabel(fig,'Position',[230 560-1*250 240 240], 'FontSize',12, 'Text', 'Controller 2: Meas. Pressure');
cg3 = uilabel(fig,'Position',[230 560-2*250 240 240], 'FontSize',12, 'Text', 'Controller 3: Meas. Pressure');

cg1 = uilabel(fig,'Position',[30 500 240 240], 'FontSize', 80, 'Text', '0.00');
cg2 = uilabel(fig,'Position',[30 500-1*250 240 240], 'FontSize',80, 'Text', '0.00');
cg3 = uilabel(fig,'Position',[30 500-2*250 240 240], 'FontSize',80, 'Text', '0.00');

sld1 = uislider(fig,...
    'Limits', [0 110],...
    'MajorTicks', 0:10:110,...
    'Position',[30 550 400 240],...
    'ValueChangedFcn',@(sld,event) updateGauge1(sld,cg1));
sld2 = uislider(fig,...
    'Limits', [0 110],...
    'MajorTicks', 0:10:110,...
    'Position',[30 550-1*250 400 240],...
    'ValueChangedFcn',@(sld,event) updateGauge2(sld,cg2));
sld3 = uislider(fig,...
    'Limits', [0 110],...
    'MajorTicks', 0:10:110,...
    'Position',[30 550-2*250 400 240],...
    'ValueChangedFcn',@(sld,event) updateGauge3(sld,cg3));

switch1 = uiswitch(fig,'toggle',...
    'Items',{'Control Off','Control On'},...    
    'Position',[490 585  20 45],...
    'ValueChangedFcn',@switchMoved1); 

switch2 = uiswitch(fig,'toggle',...
    'Items',{'Control Off','Control On'},...       
    'Position',[ 490 585-1*250 20 45],...
    'ValueChangedFcn',@switchMoved2); 

switch3 = uiswitch(fig,'toggle',...
    'Items',{'Control Off','Control On'},...    
    'Position',[ 490 585-2*250 20 45],...
    'ValueChangedFcn',@switchMoved3); 

end

% ValueChangedFcn callback
function switchMoved1(src,event)  
    switch src.Value
        case 'Control On'
            disp('Control On 1')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve1'));
            sendCommand(evalin('base','s'), evalin('base','cmd.rctrl'));
        case 'Control Off'
            disp('Control Off 1')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve1'));
            sendCommand(evalin('base','s'), evalin('base','cmd.keep'));
    end
end

% ValueChangedFcn callback
function switchMoved2(src,event)  
    switch src.Value
        case 'Control On'
            disp('Control On 2')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve2'));
            sendCommand(evalin('base','s'), evalin('base','cmd.rctrl'));
        case 'Control Off'
            disp('Control Off 2')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve2'));
            sendCommand(evalin('base','s'), evalin('base','cmd.keep'));
    end
end

% ValueChangedFcn callback
function switchMoved3(src,event)  
    switch src.Value
        case 'Control On'
            disp('Control On 3')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve3'));
            sendCommand(evalin('base','s'), evalin('base','cmd.rctrl'));
        case 'Control Off'
            disp('Control Off 3')
            sendCommand(evalin('base','s'), evalin('base','cmd.valve3'));
            sendCommand(evalin('base','s'), evalin('base','cmd.keep'));
    end
end

% Create ValueChangedFcn callback
function updateGauge1(sld1,cg)
val = round(sld1.Value*2)/2;
cg.Text = num2str( val );
setGoalPressure(evalin('base','s'),'1',val);
end

% Create ValueChangedFcn callback
function updateGauge2(sld1,cg)
val = round(sld1.Value*2)/2;
cg.Text = num2str( val );
setGoalPressure(evalin('base','s'),'2',val);
end

% Create ValueChangedFcn callback
function updateGauge3(sld1,cg)
val = round(sld1.Value*2)/2;
cg.Text = num2str( val );
setGoalPressure(evalin('base','s'),'3',val);
end

function updateGauge4(sld1,cg)
val = round(sld1.Value*2)/2;
cg.Text = num2str( val );
s = evalin('base','s');
pr = val; pr1 = pr; pr2 = pr; pr3 = pr; setGoalPressure(s,'1',pr1); setGoalPressure(s,'2',pr2); setGoalPressure(s,'3',pr3);
end

