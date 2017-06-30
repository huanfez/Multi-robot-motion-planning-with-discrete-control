% ReplottingScript
% Replot script once replanning is complete

%% Replotting 
% pause
% plot new path and obstacle
if numOFigs == 2
    figure(1)
else
    subplot(1,2,1)
end
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
% title('Planning'),
xlabel('x'),ylabel('y')
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');
if ~isempty(goalsReachedAll)
    fillGoal(goalsReachedAll,.4,gridWidth,gridLength,'m');
end
for mSub = 1:M % nested m (agent) loop
    cEnd = find(ismember(cPath{mSub},goalsLeft{mSub}),1,'last');
%     plot(xSObs{mSub},ySObs{mSub},'xr','MarkerSize',15)
    obsPlot(xSObs{mSub},ySObs{mSub},1.0,[0.6 0.6 0.6],2);
    plot(xPath{mSub}(1),yPath{mSub}(1),'o','color',colorBox(mSub),'MarkerSize',12)
    if ~isempty(cEnd)
        arrow([xPath{mSub}(1:cEnd-1)' - (mSub-2)*0.1,yPath{mSub}(1:cEnd-1)' - (mSub-2)*0.1],...
        [xPath{mSub}(2:cEnd)' - (mSub-2)*0.1,yPath{mSub}(2:cEnd)'- (mSub-2)*0.1],8,'Color',colorBox(mSub),'LineWidth',1);
    end
    rectangle('Position',[gridChunk(mSub,1)-.5, gridChunk(mSub,3)-.5, ...
        gridChunk(mSub,2)-gridChunk(mSub,1)+1, gridChunk(mSub,4)-gridChunk(mSub,3)+1],...
        'EdgeColor',colorBox(mSub),'LineWidth',3);
    IndexNum(mSub,1) = text(xPath{mSub}(1)+.15,yPath{mSub}(1)+.25,num2str(mSub));
    set(IndexNum(mSub,1),'fontsize',16)
end
hold off
if numOFigs == 2
    figure(2)
else
    title('Planning')
    set(gca,'fontsize',20)
    subplot(1,2,2)
end
hold on
% pause
%% Return to loop
