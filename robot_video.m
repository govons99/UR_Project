
% ESP variables
q1_ESP = q_ESP(:,1);
q2_ESP = q_ESP(:,2);
q3_ESP = q_ESP(:,3);

% ESPp variables
q1_ESPp = q_ESPp(:,1);
q2_ESPp = q_ESPp(:,2);
q3_ESPp = q_ESPp(:,3);

% PD variables
q1_PD = q_PD(:, 1);
q2_PD = q_PD(:, 2);
q3_PD = q_PD(:, 3);

dim = max(size(q1_ESP));

v = figure('name','video');

tit = sgtitle(v, sprintf('Time: %.2f s',t_ESP(1)));

subplot(2,2,1)
title('ESP control');
hold on; grid on; axis equal;
xlim([-0.5 3.5]);
ylim([-0.5 3.5]);
plot(0,0,'ok','MarkerFaceColor','k');
p1_ESP = animatedline('Marker','o','MarkerFaceColor','k');
p2_ESP = animatedline('Marker','o','MarkerFaceColor','k');
p3_ESP = animatedline('Marker','o','MarkerFaceColor','k');
l1_ESP = animatedline('Linewidth',2);
l2_ESP = animatedline('Linewidth',2);
l3_ESP = animatedline('Linewidth',2);
force_ESP = animatedline('linewidth', 2, 'color', 'r');
force_point_ESP = animatedline('Marker', 'd', 'color', 'r');

subplot(2,2,2)
title('ESP+ control');
hold on; grid on; axis equal;
xlim([-0.5 3.5]);
ylim([-0.5 3.5]);
plot(0,0,'ok','MarkerFaceColor','k');
p1_ESPp = animatedline('Marker','o','MarkerFaceColor','k');
p2_ESPp = animatedline('Marker','o','MarkerFaceColor','k');
p3_ESPp = animatedline('Marker','o','MarkerFaceColor','k');
l1_ESPp = animatedline('Linewidth',2);
l2_ESPp = animatedline('Linewidth',2);
l3_ESPp = animatedline('Linewidth',2);
force_ESPp = animatedline('linewidth', 2, 'color', 'r');
force_point_ESPp = animatedline('Marker', 'd', 'color', 'r');

subplot(2,2,3.5);
title('PD control');
hold on; grid on; axis equal;
xlim([-0.5 3.5]);
ylim([-0.5 3.5]);
plot(0,0,'ok','MarkerFaceColor','k');
p1_PD = animatedline('Marker','o','MarkerFaceColor','k');
p2_PD = animatedline('Marker','o','MarkerFaceColor','k');
p3_PD = animatedline('Marker','o','MarkerFaceColor','k');
l1_PD = animatedline('Linewidth',2);
l2_PD = animatedline('Linewidth',2);
l3_PD = animatedline('Linewidth',2);
force_PD = animatedline('linewidth', 2, 'color', 'r');
force_point_PD = animatedline('Marker', 'd', 'color', 'r');

for i=1:10:dim
    
    tit = sgtitle(v, sprintf('Time: %.2f s',t_ESP(i)));
    
    % ESP
    pc1_ESP = [cos(q1_ESP(i)); sin(q1_ESP(i))];
    pc2_ESP = [pc1_ESP(1)+cos(q1_ESP(i)+q2_ESP(i)); pc1_ESP(2)+sin(q1_ESP(i)+q2_ESP(i))];
    pc3_ESP = [pc2_ESP(1)+cos(q1_ESP(i)+q2_ESP(i)+q3_ESP(i)); pc2_ESP(2)+sin(q1_ESP(i)+q2_ESP(i)+q3_ESP(i))];
     
    addpoints(p1_ESP,pc1_ESP(1),pc1_ESP(2));
    addpoints(p2_ESP,pc2_ESP(1),pc2_ESP(2));

    addpoints(l1_ESP,[0 pc1_ESP(1)],[0 pc1_ESP(2)]);
    addpoints(l2_ESP,[pc1_ESP(1) pc2_ESP(1)],[pc1_ESP(2) pc2_ESP(2)]);
    addpoints(l3_ESP,[pc2_ESP(1) pc3_ESP(1)],[pc2_ESP(2) pc3_ESP(2)]);
    
    %ESP+
    pc1_ESPp = [cos(q1_ESPp(i)); sin(q1_ESPp(i))];
    pc2_ESPp = [pc1_ESPp(1)+cos(q1_ESPp(i)+q2_ESPp(i)); pc1_ESPp(2)+sin(q1_ESPp(i)+q2_ESPp(i))];
    pc3_ESPp = [pc2_ESPp(1)+cos(q1_ESPp(i)+q2_ESPp(i)+q3_ESPp(i)); pc2_ESPp(2)+sin(q1_ESPp(i)+q2_ESPp(i)+q3_ESPp(i))];
    
    addpoints(p1_ESPp,pc1_ESPp(1),pc1_ESPp(2));
    addpoints(p2_ESPp,pc2_ESPp(1),pc2_ESPp(2)); 

    addpoints(l1_ESPp,[0 pc1_ESPp(1)],[0 pc1_ESPp(2)]);
    addpoints(l2_ESPp,[pc1_ESPp(1) pc2_ESPp(1)],[pc1_ESPp(2) pc2_ESPp(2)]);
    addpoints(l3_ESPp,[pc2_ESPp(1) pc3_ESPp(1)],[pc2_ESPp(2) pc3_ESPp(2)]);
    
    %PD
    pc1_PD = [cos(q1_PD(i)); sin(q1_PD(i))];
    pc2_PD = [pc1_PD(1)+cos(q1_PD(i)+q2_PD(i)); pc1_PD(2)+sin(q1_PD(i)+q2_PD(i))];
    pc3_PD = [pc2_PD(1)+cos(q1_PD(i)+q2_PD(i)+q3_PD(i)); pc2_PD(2)+sin(q1_PD(i)+q2_PD(i)+q3_PD(i))];
    
    addpoints(p1_PD,pc1_PD(1),pc1_PD(2));
    addpoints(p2_PD,pc2_PD(1),pc2_PD(2)); 

    addpoints(l1_PD,[0 pc1_PD(1)],[0 pc1_PD(2)]);
    addpoints(l2_PD,[pc1_PD(1) pc2_PD(1)],[pc1_PD(2) pc2_PD(2)]);
    addpoints(l3_PD,[pc2_PD(1) pc3_PD(1)],[pc2_PD(2) pc3_PD(2)]);
    
    if t_ESP(i) <= T2 && t_ESP(i) >= T1
    
        % force vector ESP
        switch link

            case 1
                force_contact_ESP = 0.5*[pc1_ESP(1); pc1_ESP(2)];
                addpoints(force_point_ESP, force_contact_ESP(1), force_contact_ESP(2));

                xf = force_contact_ESP(1);
                xi = xf - 0.5*sin(q0(1));
                yf = force_contact_ESP(2);
                yi = yf + 0.5*cos(q0(1));
                addpoints(force_ESP, [xi xf], [yi yf]);

            case 2
                force_contact_ESP = [pc1_ESP(1)+0.5*cos(q1_ESP(i)+q2_ESP(i)); pc1_ESP(2)+0.5*sin(q1_ESP(i)+q2_ESP(i))];
                addpoints(force_point_ESP, force_contact_ESP(1), force_contact_ESP(2));

                xf = force_contact_ESP(1);
                xi = xf - 0.5*sin(q0(1) + q0(2));
                yf = force_contact_ESP(2);
                yi = yf + 0.5*cos(q0(1) + q0(2));
                addpoints(force_ESP, [xi xf], [yi yf]);

            case 3
                force_contact_ESP = [pc2_ESP(1)+0.5*cos(q1_ESP(i)+q2_ESP(i)+q3_ESP(i)); pc2_ESP(2)+0.5*sin(q1_ESP(i)+q2_ESP(i)+q3_ESP(i))];
                addpoints(force_point_ESP, force_contact_ESP(1), force_contact_ESP(2));

                xf = force_contact_ESP(1);
                xi = xf - 0.5*sin(q0(1) + q0(2) + q0(3));
                yf = force_contact_ESP(2);
                yi = yf + 0.5*cos(q0(1) + q0(2) + q0(3));
                addpoints(force_ESP, [xi xf], [yi yf]);
        end  

        % force vector ESP+
        switch link

            case 1
                force_contact_ESPp = 0.5*[pc1_ESPp(1); pc1_ESPp(2)];
                addpoints(force_point_ESPp, force_contact_ESPp(1), force_contact_ESPp(2));

                xf = force_contact_ESPp(1);
                xi = xf - 0.5*sin(q0(1));
                yf = force_contact_ESPp(2);
                yi = yf + 0.5*cos(q0(1));
                addpoints(force_ESPp, [xi xf], [yi yf]);

            case 2
                force_contact_ESPp = [pc1_ESPp(1)+0.5*cos(q1_ESPp(i)+q2_ESPp(i)); pc1_ESPp(2)+0.5*sin(q1_ESPp(i)+q2_ESPp(i))];
                addpoints(force_point_ESPp, force_contact_ESPp(1), force_contact_ESPp(2));

                xf = force_contact_ESPp(1);
                xi = xf - 0.5*sin(q0(1) + q0(2));
                yf = force_contact_ESPp(2);
                yi = yf + 0.5*cos(q0(1) + q0(2));
                addpoints(force_ESPp, [xi xf], [yi yf]);

            case 3
                force_contact_ESPp = [pc2_ESPp(1)+0.5*cos(q1_ESPp(i)+q2_ESPp(i)+q3_ESPp(i)); pc2_ESPp(2)+0.5*sin(q1_ESPp(i)+q2_ESPp(i)+q3_ESPp(i))];
                addpoints(force_point_ESPp, force_contact_ESPp(1), force_contact_ESPp(2));

                xf = force_contact_ESPp(1);
                xi = xf - 0.5*sin(q0(1) + q0(2) + q0(3));
                yf = force_contact_ESPp(2);
                yi = yf + 0.5*cos(q0(1) + q0(2) + q0(3));
                addpoints(force_ESPp, [xi xf], [yi yf]);
        end
        
        % force vector PD
        switch link

            case 1
                force_contact_PD = 0.5*[pc1_PD(1); pc1_PD(2)];
                addpoints(force_point_PD, force_contact_PD(1), force_contact_PD(2));

                xf = force_contact_PD(1);
                xi = xf - 0.5*sin(q0(1));
                yf = force_contact_PD(2);
                yi = yf + 0.5*cos(q0(1));
                addpoints(force_PD, [xi xf], [yi yf]);

            case 2
                force_contact_PD = [pc1_PD(1)+0.5*cos(q1_PD(i)+q2_PD(i)); pc1_PD(2)+0.5*sin(q1_PD(i)+q2_PD(i))];
                addpoints(force_point_PD, force_contact_PD(1), force_contact_PD(2));

                xf = force_contact_PD(1);
                xi = xf - 0.5*sin(q0(1) + q0(2));
                yf = force_contact_PD(2);
                yi = yf + 0.5*cos(q0(1) + q0(2));
                addpoints(force_PD, [xi xf], [yi yf]);

            case 3
                force_contact_PD = [pc2_PD(1)+0.5*cos(q1_PD(i)+q2_PD(i)+q3_PD(i)); pc2_PD(2)+0.5*sin(q1_PD(i)+q2_PD(i)+q3_PD(i))];
                addpoints(force_point_PD, force_contact_PD(1), force_contact_PD(2));

                xf = force_contact_PD(1);
                xi = xf - 0.5*sin(q0(1) + q0(2) + q0(3));
                yf = force_contact_PD(2);
                yi = yf + 0.5*cos(q0(1) + q0(2) + q0(3));
                addpoints(force_PD, [xi xf], [yi yf]);
        end
    
    end
    
    pause(0.0001);
    
    if i~=dim
        
        clearpoints(p1_ESP); clearpoints(p2_ESP);
        clearpoints(p3_ESP); clearpoints(l1_ESP);
        clearpoints(l2_ESP); clearpoints(l3_ESP);
        clearpoints(force_point_ESP); clearpoints(force_ESP);
        
        clearpoints(p1_ESPp); clearpoints(p2_ESPp);
        clearpoints(p3_ESPp); clearpoints(l1_ESPp);
        clearpoints(l2_ESPp); clearpoints(l3_ESPp);
        clearpoints(force_point_ESPp); clearpoints(force_ESPp);
        
        clearpoints(p1_PD); clearpoints(p2_PD);
        clearpoints(p3_PD); clearpoints(l1_PD);
        clearpoints(l2_PD); clearpoints(l3_PD);
        clearpoints(force_point_PD); clearpoints(force_PD);
        
    end
       
end

