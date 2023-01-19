% Disclaimer:
% --------------
% THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
% SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
% IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
% IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

function createAcclPlot(YMatrix1)

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot
plot1 = plot(YMatrix1,'Parent',axes1);
set(plot1(1),'DisplayName','X');
set(plot1(2),'DisplayName','Y');
set(plot1(3),'DisplayName','Z');

% Set ylim
ylim([-3000 3000]);

% Create ylabel
ylabel({'Linear Acceleration (mG)'});

% Create xlabel
xlabel({'Sample (n)'});

% Create title
title({'Accl'});

box(axes1,'on');
% Create legend
legend(axes1,'show');

