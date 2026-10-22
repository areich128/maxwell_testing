function ok = rebuild_sfun_mex(names)
% rebuild_sfun_mex  Recompile S-Function Builder blocks without the GUI.
%
% Reads the SOURCEFILES list that the S-Function Builder wrote into
% mexFiles/<name>.c and calls mex with the same block files, flight sources,
% include paths and flags the Builder uses ('-g', '-R2018a',
% '-DUSE_PUBLISHED_ONLY'). Only the .mexw64 binaries change; the block .c,
% wrapper .c, .tlc and the library are not touched. Use it when flight C
% sources changed but the block ports did not. If a block's ports change,
% press Build in its S-Function Builder instead.
%
% Run from simulink_test\sFunctionLibrary:
%   rebuild_sfun_mex            % the 20 blocks that compile att_det.c / ref_rotation.c
%   rebuild_sfun_mex({'igrf_eci_sfun'})

if nargin < 1
    names = {'a_initial','calc_q_er','calculateOmega','control_fdbk', ...
        'create_des_rates_CSAC','create_des_rates_DCOMM','create_desRN', ...
        'create_phicone_des_q','create_sun_des_q','dcm_2_q','est_quest_rp', ...
        'gs_angle_distance','gs_to_sc_vec','igrf_eci_sfun','p_derivative', ...
        'q_2_dcm','q_from_2uvecs','sc_to_gs_unitvec','sc_to_gs_vec','sun_eci_sfun'};
end

here = pwd;
[~, folder] = fileparts(here);
assert(strcmp(folder, 'sFunctionLibrary'), ...
    'Run this from simulink_test\\sFunctionLibrary (blocks use paths relative to it).');
outdir = fullfile(here, 'mexFiles');

clear mex
ok = false(numel(names), 1);

for k = 1:numel(names)
    n = names{k};
    blockC = fullfile(outdir, [n '.c']);
    wrapC  = fullfile(outdir, [n '_wrapper.c']);
    if ~isfile(blockC) || ~isfile(wrapC)
        fprintf(2, '%-24s SKIPPED: %s or its wrapper not found\n', n, blockC);
        continue
    end

    txt = fileread(blockC);
    tok = regexp(txt, '#define\s+SOURCEFILES\s+"([^"]*)"', 'tokens', 'once');
    if isempty(tok)
        fprintf(2, '%-24s SKIPPED: no SOURCEFILES line in %s\n', n, blockC);
        continue
    end

    srcs = {blockC, wrapC};
    incs = {['-I' here]};
    parts = strsplit(tok{1}, '__SFB__');
    for p = 1:numel(parts)
        s = strtrim(parts{p});
        if isempty(s), continue, end
        if startsWith(s, 'INC_PATH ')
            incs{end+1} = ['-I' resolve(strtrim(s(10:end)), here)]; %#ok<AGROW>
        elseif startsWith(s, 'SRC_PATH ') || startsWith(s, 'LIB_PATH ')
            % search paths only, nothing to compile
        elseif endsWith(s, {'.c', '.lib', '.a', '.obj'})
            srcs{end+1} = resolve(s, here); %#ok<AGROW>
        end
    end

    try
        evalc('mex(''-g'', ''-R2018a'', ''-DUSE_PUBLISHED_ONLY'', srcs{:}, incs{:}, ''-outdir'', outdir)');
        ok(k) = true;
        fprintf('%-24s rebuilt (%d flight sources)\n', n, numel(srcs) - 2);
    catch err
        fprintf(2, '%-24s FAILED: %s\n', n, err.message);
    end
end

fprintf('\n%d of %d blocks rebuilt.\n', sum(ok), numel(ok));
if any(~ok)
    fprintf(2, 'Not rebuilt: %s\n', strjoin(names(~ok), ', '));
end
clear mex
end

function p = resolve(s, here)
% absolute paths (C:\...) are kept; relative ones are taken from sFunctionLibrary
if ~isempty(regexp(s, '^[A-Za-z]:[\\/]', 'once'))
    p = s;
else
    p = fullfile(here, s);
end
end