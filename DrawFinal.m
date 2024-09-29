figure(1)
set(0,'defaultfigurecolor','w')


for i=1:500:length(Pcur.data(1,1,:))
    hold on;
    axis equal;
    for k=1:length(p_obstacle(:,1))
        line([potl(k,1),potu(k,1)],[potl(k,2),potu(k,2)],'Color','black','LineStyle','-','Linewidth',1.5);
        line([potl(k,1),potd(k,1)],[potl(k,2),potd(k,2)],'Color','black','LineStyle','-','Linewidth',1.5);
        line([potd(k,1),potu(k,1)],[potd(k,2),potu(k,2)],'Color','black','LineStyle','-','Linewidth',1.5);

        line([pol0(k,1),por0(k,1)],[pol0(k,2),por0(k,2)],'Color','#77AC30','LineStyle','-.','Linewidth',1.5);
        line([pol1(k,1),potu(k,1)],[pol1(k,2),potu(k,2)],'Color','#77AC30','LineStyle','-.','Linewidth',1.5);
        line([por1(k,1),potd(k,1)],[por1(k,2),potd(k,2)],'Color','#77AC30','LineStyle','-.','Linewidth',1.5);

    end
    for k=1:length(p_left(:,1))
        line([p_left(k,1),p_right(k,1)],[p_left(k,2),p_right(k,2)],'Color','blue','LineStyle','--','Linewidth',1);
    end
    plot(p_left(:,1),p_left(:,2),'k','Linewidth',1.5);
    plot(p_right(:,1),p_right(:,2),'k','Linewidth',1.5);
    for k=1:length(p_obstacle(:,1))
        alpha = 0:pi/20:2*pi;
        x = p_obstacle(k,1) +  (ro(k)-0.02)*cos(alpha);
        y = p_obstacle(k,2) +  (ro(k)-0.02)*sin(alpha);
        fill(x,y,[0.9290 0.6940 0.1250],'LineWidth',0.01,'EdgeColor',[0.9290 0.6940 0.1250])
    end
    for j=1:N
        fill_r(Pcur.data(j,1:2,i),rs,'r',0.1);
    end
    axis([-5,35,-10,10]);
    pause(0.005);
    hold off
    plot(0,0);
end