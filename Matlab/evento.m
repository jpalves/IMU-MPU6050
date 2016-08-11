function evento(hObject,event,s)
    v = sscanf(fgets(s),'%f');
         
    
    global h2 versor
    rotMat = quat2dcm(v.');% transpose because ahrs provides Earth relative to sensor
    o_v =  rotMat*versor;
    
    plot3([0 ,o_v(1,1)],[0 ,o_v(1,2)],[0 ,o_v(1,3)],'color',[1 0 0],'Parent',h2)
    hold on
    plot3([0 ,o_v(2,1)],[0 ,o_v(2,2)],[0 ,o_v(2,3)],'color',[0 1 0],'Parent',h2)
    plot3([0 ,o_v(3,1)],[0 ,o_v(3,2)],[0 ,o_v(3,3)],'color',[0 0 1],'Parent',h2)
        
    %xlabel('x')
    %ylabel('y')
    hold off
    grid on
    axis([-1 1 -1 1 -1 1])
    drawnow
    