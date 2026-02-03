function data = csv_topics_to_d(ulgFileName)
%------------------------------------------
% Convert CSV files (from ulog2csv) to MATLAB struct
%------------------------------------------

    [filepath, name, ~] = fileparts(ulgFileName);
    all_topics = dir(fullfile(filepath, [name '_*.csv'])); % Use absolute path

    if isempty(all_topics)
        error('❌ No CSV files found in %s', filepath);
    end

    h = waitbar(0, 'Converting .ulg CSV files to .mat...');
    steps = length(all_topics);
    data = struct();

    for i = 1:steps
        waitbar(i / steps);

        tmp = strsplit(all_topics(i).name(1:end-4), [name '_']);
        topic_i = tmp{2};

        % Read CSV file (full path)
        filePath = fullfile(filepath, all_topics(i).name);
        data.(topic_i) = readtable(filePath);

        % Special handling for GPS timestamp
        if contains(all_topics(i).name, 'vehicle_gps_position_0.csv')
            data.(topic_i).time_utc_usec = typecast(data.(topic_i).time_utc_usec, 'uint64');
        end

        % Read first line as column description
        fid = fopen(filePath);
        tline = fgetl(fid);
        fclose(fid);
        data.(topic_i).Properties.VariableDescriptions = strsplit(tline, ',');
    end

    close(h);
    disp(['Parsed ' num2str(steps) ' topics successfully from ' filepath]);
end