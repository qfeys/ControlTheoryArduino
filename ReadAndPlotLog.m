function rec = ReadAndPlotLog(filename)

    rec = readlog(['C:\Brol\QRobotics\Windows\logs\', filename, '.xml']);

%     v_filtered = (rec.data(1:end-2,2)+ 2*rec.data(2:end-1,2)+ rec.data(3:end,2))./4;
%     p_start = 209;
%     p_end = 290;
%     p_start = 1069;
%     p_end = 1170;
%     v_up = v_filtered(p_start:p_end);
%     t_up = rec.data(p_start:p_end,1);
%     t_up = t_up - t_up(1);
%     GauseNewtonOptimalisation(t_up,v_up);

    %figure
    %hold on
    % v
    %plot(rec.data(:,1),rec.data(:,2))
    % v - filtered
    %plot(rec.data(2:end-1,1),(rec.data(1:end-2,2)+ 2*rec.data(2:end-1,2)+ rec.data(3:end,2))./4)
    % u
    %plot(rec.data(:,1),rec.data(:,6))
end