% ReplottingScript
% Replot script once replanning is complete

%% Replotting 
% pause
% plot new path and obstacle
if numOFigs == 2
    figure(2)
else
    subplot(1,2,1)
end
% hold off
% plot(xGrid,yGrid,'k')
% axis equal
% axis([0 gridWidth+1 0 gridLength+1])
% % title('Planning'),
% xlabel('x'),ylabel('y')
% hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
% goalPlot(xGoal,yGoal,.4,'m');
% if ~isempty(goalsReachedAll)
%     fillGoal(goalsReachedAll,.4,gridWidth,gridLength,'m');
% end
for mSub = 1:M % nested m (agent) loop
    cEnd = find(ismember(cPath{mSub},goalsLeft{mSub}),1,'last');
%     plot(xSObs{mSub},ySObs{mSub},'xr','MarkerSize',15)
%     obsPlot(xSObs{mSub},ySObs{mSub},.3,'r',3);
%     plot(xPath{mSub}(1),yPath{mSub}(1),'ko',xPath{mSub}(cEnd),yPath{mSub}(cEnd),'k^',...
%         'MarkerSize',12)
    if ~isempty(cEnd)
    delete(hArrow{mSub})
    hArrow{mSub} = arrow([xPath{mSub}(1:cEnd-1)',yPath{mSub}(1:cEnd-1)'],...
        [xPath{mSub}(2:cEnd)',yPath{mSub}(2:cEnd)'],10,'EdgeColor',myColors(mSub),'FaceColor',myColors(mSub));
    end
%     rectangle('Position',[gridChunk(mSub,1)-.5, gridChunk(mSub,3)-.5, ...
%         gridChunk(mSub,2)-gridChunk(mSub,1)+1, gridChunk(mSub,4)-gridChunk(mSub,3)+1],...
%         'LineWidth',6);
end
hold off
if numOFigs == 2
    figure(2)
else
    title('Planning')
    subplot(1,2,2)
end
hold on
% pause
%% Return to loop