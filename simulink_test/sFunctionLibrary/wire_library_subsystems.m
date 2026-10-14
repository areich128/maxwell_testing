function wire_library_subsystems()
% Expose every unconnected port of the inner flight S-functions as an Inport or
% Outport of its library subsystem, with the S-function's own name, type and size.
% Existing ports and their numbering are left unchanged; new ones are appended.

    lib = 'sFunctionLibrary';
    libDir = fileparts(which([lib '.slx']));
    mexDir = fullfile(libDir, 'mexFiles');
    load_system(lib);
    set_param(lib, 'Lock', 'off');

    subs = {'control_fdbk',        'control_fdbk';
            'est_quest_rp',        'est_quest_rp';
            'createDes_RN',        'create_desRN';
            'WheelTorque_Command', 'S-Function Builder';
            'q_from_2uvecs',       'q_from_2uvecs'};
    dt = containers.Map({'real32_T','real_T','int32_T','uint32_T','int8_T','uint8_T','boolean_T'}, ...
                        {'single','double','int32','uint32','int8','uint8','boolean'});
    nAdded = 0;

    for s = 1:size(subs, 1)
        sys = [lib '/' subs{s,1}];
        inner = subs{s,2};
        blk = [sys '/' inner];
        fn = get_param(blk, 'FunctionName');
        P = parsePorts(fileread(fullfile(mexDir, [fn '.c'])));
        ph = get_param(blk, 'PortHandles');
        assert(numel(ph.Inport) == numel(P.in) && numel(ph.Outport) == numel(P.out), ...
            '%s has stale ports (%d/%d vs mex %d/%d): rebuild it first', blk, ...
            numel(ph.Inport), numel(ph.Outport), numel(P.in), numel(P.out));
        fprintf('%s (%s)\n', sys, fn);

        for k = 1:numel(P.in)
            lh = get_param(ph.Inport(k), 'Line');
            if lh ~= -1 && get_param(lh, 'SrcBlockHandle') ~= -1, continue; end
            if lh ~= -1, delete_line(lh); end
            nm = uniqueName(sys, P.in(k).name);
            pp = get_param(ph.Inport(k), 'Position');
            add_block('simulink/Sources/In1', [sys '/' nm], 'Position', [pp(1)-170 pp(2)-7 pp(1)-140 pp(2)+7], ...
                'OutDataTypeStr', dt(P.in(k).dtype), 'PortDimensions', mat2str([P.in(k).w P.in(k).c]));
            add_line(sys, [nm '/1'], sprintf('%s/%d', inner, k), 'autorouting', 'on');
            fprintf('   + Inport  %-20s -> in %2d  %s [%d,%d]  (subsystem port %s)\n', nm, k, ...
                dt(P.in(k).dtype), P.in(k).w, P.in(k).c, get_param([sys '/' nm], 'Port'));
            nAdded = nAdded + 1;
        end

        for k = 1:numel(P.out)
            lh = get_param(ph.Outport(k), 'Line');
            if lh ~= -1 && any(get_param(lh, 'DstBlockHandle') ~= -1), continue; end
            if lh ~= -1, delete_line(lh); end
            nm = uniqueName(sys, P.out(k).name);
            pp = get_param(ph.Outport(k), 'Position');
            add_block('simulink/Sinks/Out1', [sys '/' nm], 'Position', [pp(1)+140 pp(2)-7 pp(1)+170 pp(2)+7]);
            add_line(sys, sprintf('%s/%d', inner, k), [nm '/1'], 'autorouting', 'on');
            fprintf('   + Outport %-20s <- out %d  %s [%d,%d]  (subsystem port %s)\n', nm, k, ...
                dt(P.out(k).dtype), P.out(k).w, P.out(k).c, get_param([sys '/' nm], 'Port'));
            nAdded = nAdded + 1;
        end
    end

    save_system(lib);
    fprintf('\n%d ports added, %s saved\n', nAdded, lib);
end

function P = parsePorts(src)
    P.in = getPorts(src, 'IN_PORT', 'INPUT');
    P.out = getPorts(src, 'OUT_PORT', 'OUTPUT');
end

function ports = getPorts(src, namePfx, pfx)
    n = str2double(regexp(src, ['#define NUM_' pfx 'S\s+(\d+)'], 'tokens', 'once'));
    ports = struct('name', {}, 'dtype', {}, 'w', {}, 'c', {});
    for k = 0:n-1
        g = @(key) regexp(src, sprintf('#define %s\\s+(\\w+)', sprintf(key, k)), 'tokens', 'once');
        ports(k+1).name  = char(g([namePfx '_%d_NAME']));
        ports(k+1).dtype = char(g([pfx '_%d_DTYPE']));
        ports(k+1).w     = str2double(g([pfx '_%d_WIDTH']));
        ports(k+1).c     = str2double(g([pfx '_DIMS_%d_COL']));
    end
end

function nm = uniqueName(sys, nm)
    base = nm; i = 1;
    while ~isempty(find_system(sys, 'SearchDepth', 1, 'Name', nm))
        nm = sprintf('%s_%d', base, i); i = i + 1;
    end
end