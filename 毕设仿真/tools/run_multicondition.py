"""Frozen, single-attempt physical batch with explicit published control provenance.

prepare writes declarations only. run requires a committed-source receipt and
verified deployment, and never retries an algorithm failure. audit reconciles
existing evidence without launching physics. All scene futures remain world-only.
"""
from pathlib import Path
import argparse
import datetime
import gzip
import hashlib
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time
import traceback
import uuid

CONTROL_COMMIT = '748b27e28be42ad294769105c5c899d3e9ecfb25'
CATEGORIES = ('complete_function_success', 'correct_safe_exit', 'coverage_failure',
              'algorithm_control_or_physical_failure', 'environment_or_insufficient_evidence')
REQUIRED_CASE_FIELDS = {'case_id', 'scenario', 'kind', 'parameters', 'trigger',
                        'coverage', 'expected', 'seed', 'initial', 'budget', 'auditor_version'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp.' + uuid.uuid4().hex)
    with temporary.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def validate_matrix(matrix):
    cases = matrix.get('cases', [])
    expected = [f'M{i:02d}' for i in range(1, 21)]
    if len(cases) != 20 or [c.get('case_id') for c in cases] != expected:
        raise ValueError('Freeze exactly twenty unique ordered cases M01..M20')
    for case in cases:
        if not REQUIRED_CASE_FIELDS <= case.keys():
            raise ValueError('Incomplete case declaration: ' + case['case_id'])
        if case['scenario'] != 'multi_' + case['case_id'].lower():
            raise ValueError('Case/scenario mismatch: ' + case['case_id'])


def prepare(root, output, matrix):
    root, output = Path(root), Path(output)
    if output.exists():
        raise FileExistsError('Preserve existing evidence; choose a new output')
    validate_matrix(matrix)
    profile_path = root/'config/experiments/planar_reacquisition_v1.json'
    baseline = read_json(profile_path)
    package_sources = read_json(root/'simulation/operator_package/source_manifest.json')['source_sha256']
    if package_sources.get('config/experiments/planar_reacquisition_v1.json') != sha(profile_path):
        raise ValueError('Accepted original control profile changed')
    output.mkdir(parents=True, exist_ok=False)
    (output/'profiles').mkdir()
    frozen = dict(matrix, planned_cases=20, control_commit=CONTROL_COMMIT,
                  baseline_profile_sha256=sha(profile_path))
    save(output/'matrix.json', frozen)
    hashes = {'matrix.json': sha(output/'matrix.json')}
    for case in frozen['cases']:
        cid, scenario = case['case_id'], case['scenario']
        folder = output/'cases'/cid
        folder.mkdir(parents=True)
        save(folder/'case.json', case)
        # No future formula, sequence, trigger, or random seed reaches the planner.
        cfg = dict(baseline)
        cfg.pop('reacquisition_trial', None)
        cfg.update(default_scenario=scenario, validation_scenarios=[scenario],
                   scenarios={scenario: case.get('name', case['kind'])},
                   validation_expectations={scenario: case['expected']},
                   multicondition_case={'case_id': cid, 'scenario': scenario})
        profile = output/'profiles'/f'{cid}.json'
        save(profile, cfg)
        for path in (folder/'case.json', profile):
            hashes[str(path.relative_to(output))] = sha(path)
    save(output/'freeze_sha256.json', dict(files=hashes, created_utc=stamp()))
    write_index(output, [], 'PREPARED')
    return frozen


def verify_freeze(output):
    output = Path(output)
    manifest = read_json(output/'freeze_sha256.json')['files']
    if set(manifest) != {'matrix.json', *[f'profiles/M{i:02d}.json' for i in range(1,21)],
                         *[f'cases/M{i:02d}/case.json' for i in range(1,21)]}:
        raise ValueError('Incomplete freeze manifest')
    for name, digest in manifest.items():
        if not (output/name).is_file() or sha(output/name) != digest:
            raise ValueError('Frozen evidence changed: ' + name)
    matrix = read_json(output/'matrix.json')
    validate_matrix(matrix)
    for case in matrix['cases']:
        if read_json(output/'cases'/case['case_id']/'case.json') != case:
            raise ValueError('Case differs from frozen matrix')
    return matrix


def write_index(output, attempts, stage):
    case_ids = {v['case_id'] for v in attempts}
    counts = {c: sum(v['category'] == c for v in attempts) for c in CATEGORIES}
    index = dict(planned_cases=20, attempted_cases=len(case_ids),
                 unattempted_cases=20-len(case_ids), actual_attempts=len(attempts),
                 stage=stage, counts=counts, attempts=attempts, updated_utc=stamp())
    save(Path(output)/'result_index.json', index)
    return index


def read_index(output):
    index = read_json(Path(output)/'result_index.json')
    attempts = index.get('attempts', [])
    ids = [row.get('case_id') for row in attempts]
    expected = {f'M{i:02d}' for i in range(1,21)}
    if (index.get('planned_cases') != 20 or len(ids) != len(set(ids)) or
            not set(ids) <= expected or index.get('attempted_cases') != len(ids) or
            index.get('unattempted_cases') != 20-len(ids) or
            index.get('actual_attempts') != len(attempts)):
        raise ValueError('Attempt denominator/index is corrupt')
    return index


def begin_attempt(output, case_id, run_commit, *, control_commit=CONTROL_COMMIT):
    output = Path(output)
    matrix = verify_freeze(output)
    if case_id not in {c['case_id'] for c in matrix['cases']}:
        raise ValueError('Undeclared case')
    if not re.fullmatch('[0-9a-f]{40}', run_commit):
        raise ValueError('A fixed implementation commit is required')
    index = read_index(output)
    if any(v['case_id'] == case_id for v in index['attempts']):
        raise FileExistsError('Case already attempted; algorithm results cannot be replaced')
    directory = output/'cases'/case_id/'attempt_001'
    directory.mkdir(exist_ok=False)
    row = dict(case_id=case_id, attempt=1, attempt_dir=str(directory.resolve()),
               category='in_progress', started_utc=stamp(), run_commit=run_commit,
               control_commit=control_commit)
    save(directory/'attempt.json', row)
    write_index(output, [*index['attempts'], row], 'RUNNING')
    return row


def finish_attempt(output, row):
    index = read_index(output)
    existing = [v for v in index['attempts'] if v['case_id'] == row['case_id']]
    if len(existing) != 1 or existing[0]['category'] != 'in_progress':
        raise FileExistsError('Finalized attempts are immutable')
    if row['category'] not in CATEGORIES:
        raise ValueError('Unknown outcome category')
    row = dict(row, finished_utc=stamp())
    save(Path(row['attempt_dir'])/'result.json', row)
    attempts = [row if v['case_id'] == row['case_id'] else v for v in index['attempts']]
    return write_index(output, attempts, 'COMPLETE' if len(attempts) == 20 else 'RUNNING')


def _load_tool(root, name):
    spec = importlib.util.spec_from_file_location('_mc_' + name, Path(root)/'tools'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_history_check(run):
    try:
        return _load_tool(Path(__file__).resolve().parents[1], 'audit_multicondition').history_evidence(Path(run))
    except Exception as exc:
        return dict(passed=False, error=repr(exc))


def compress_raw(results):
    results = Path(results)
    raw, packed = results/'model_states.jsonl', results/'model_states.jsonl.gz'
    if not raw.exists():
        if packed.exists() and (results/'raw_compression.json').exists():
            receipt = read_json(results/'raw_compression.json')
            with gzip.open(packed, 'rb') as stream:
                h = hashlib.sha256()
                for block in iter(lambda: stream.read(1048576), b''): h.update(block)
            if h.hexdigest() != receipt['sha256_uncompressed']:
                raise ValueError('Compressed physical evidence changed')
            return receipt
        raise FileNotFoundError('No original physical trajectory')
    if packed.exists():
        raise FileExistsError('Never overwrite archived raw evidence')
    digest, size = sha(raw), raw.stat().st_size
    with raw.open('rb') as src, packed.open('xb') as target:
        with gzip.GzipFile(filename='', fileobj=target, mode='wb', compresslevel=6, mtime=0) as dst:
            shutil.copyfileobj(src, dst)
    h = hashlib.sha256()
    with gzip.open(packed, 'rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''): h.update(block)
    if h.hexdigest() != digest:
        raise ValueError('Lossless physical evidence verification failed')
    receipt = dict(verified_lossless=True, sha256_uncompressed=digest,
                   bytes_uncompressed=size, bytes_compressed=packed.stat().st_size,
                   sha256_compressed=sha(packed))
    save(results/'raw_compression.json', receipt)
    raw.unlink()
    return receipt


def archive_raw(run, attempt_dir):
    """Retain full-frequency runtime/results and independently read every member."""
    run, attempt_dir = Path(run), Path(attempt_dir)
    archive = attempt_dir/'raw_evidence.tar.gz'
    if archive.exists(): raise FileExistsError('Raw archive already exists')
    files = sorted(p for directory in ('results', 'runtime') for p in (run/directory).rglob('*')
                   if p.is_file() and not p.is_symlink())
    manifest = {str(p.relative_to(run)): dict(sha256=sha(p), bytes=p.stat().st_size) for p in files}
    with tarfile.open(archive, 'x:gz', compresslevel=6) as tar:
        for path in files: tar.add(path, arcname=str(path.relative_to(run)), recursive=False)
    with tarfile.open(archive, 'r:gz') as tar:
        members = tar.getmembers()
        if len(members) != len(manifest): raise ValueError('Archive membership mismatch')
        for member in members:
            stream = tar.extractfile(member)
            h = hashlib.sha256()
            for block in iter(lambda: stream.read(1048576), b''): h.update(block)
            if h.hexdigest() != manifest[member.name]['sha256']:
                raise ValueError('Raw archive readback failed')
    receipt = dict(path=str(archive), files=manifest, sha256=sha(archive), bytes=archive.stat().st_size,
                   verified_lossless=True, offsite_backup=False, raw_removed=False)
    save(attempt_dir/'raw_archive_receipt.json', receipt)
    return receipt


def classify(audit, status, closed, integrity_ok, history_ok, supervisor_exit_code):
    if (not integrity_ok or not history_ok or not audit or
            closed.get('all_owned_cycle_processes_stopped') is not True or
            status.get('armed_at_save') is not False or not status):
        return 'environment_or_insufficient_evidence'
    if not all(isinstance(audit.get(key), dict) and isinstance(audit[key].get('passed'), bool)
               for key in ('physics', 'safety', 'coverage')) or not isinstance(audit.get('landing_objective'), dict):
        return 'environment_or_insufficient_evidence'
    category = audit.get('classification')
    category = {'landing': 'complete_function_success', 'safe_exit': 'correct_safe_exit',
                'control_physical_audit_failure': 'algorithm_control_or_physical_failure',
                'environment_evidence': 'environment_or_insufficient_evidence'}.get(category, category)
    if category not in CATEGORIES: return 'environment_or_insufficient_evidence'
    if category in ('complete_function_success', 'correct_safe_exit') and (
            audit['safety']['passed'] is not True or audit['physics']['passed'] is not True or audit['coverage']['passed'] is not True):
        return 'algorithm_control_or_physical_failure'
    if category == 'complete_function_success':
        if audit.get('passed') is not True or audit['landing_objective'].get('achieved') is not True:
            return 'algorithm_control_or_physical_failure'
        if supervisor_exit_code != 0 or status.get('completed') is not True or status.get('final_mode') != 'LAND':
            return 'algorithm_control_or_physical_failure'
    return category


def load_package_manifest(package, expected_sha256):
    # The original manifest remains byte-identical for the accepted demo entry.
    # Multicondition tools are attested through their own deployment manifest.
    path = Path(package)/'multicondition_source_manifest.json'
    if sha(path) != expected_sha256:
        raise ValueError('Multicondition package manifest changed')
    manifest = read_json(path)['source_sha256']
    if not manifest:
        raise ValueError('Empty multicondition package source manifest')
    return manifest


def verify_control_revision(receipt):
    control = receipt.get('control_commit')
    if control == CONTROL_COMMIT:
        return control
    changes = receipt.get('control_changes', {})
    if (not re.fullmatch('[0-9a-f]{40}', str(control)) or control != receipt.get('run_commit')
            or receipt.get('control_baseline_commit') != CONTROL_COMMIT or not changes):
        raise ValueError('Changed control requires its actual published revision and baseline')
    sources = receipt.get('source_sha256', {})
    for name, digest in changes.items():
        if (not (name.startswith('landing/') or name == 'simulation/operator_package/operator_multicondition_backend.py')
                or not re.fullmatch('[0-9a-f]{64}', str(digest)) or sources.get(name) != digest):
            raise ValueError('Control change missing committed-source hash: ' + name)
    return control


def verify_receipt(root, output, base, package):
    root, output, base, package = map(Path, (root, output, base, package))
    receipt = read_json(output/'code_receipt.json')
    if not re.fullmatch('[0-9a-f]{40}', receipt.get('run_commit', '')):
        raise ValueError('Require a published implementation commit receipt')
    verify_control_revision(receipt)
    sources = receipt.get('source_sha256', {})
    required = ('tools/run_multicondition.py', 'tools/multicondition_scene.py',
                'tools/audit_multicondition.py', 'tools/multicondition_bridge.py',
                'simulation/operator_package/operator_multicondition_supervisor.py',
                'simulation/operator_package/operator_multicondition_backend.py',
                'config/experiments/multicondition_matrix_v1.json')
    if not sources or not all(n in sources for n in required):
        raise ValueError('Incomplete committed-source receipt')
    for name, digest in sources.items():
        if not (root/name).is_file() or sha(root/name) != digest:
            raise ValueError('Committed source changed: ' + name)
    manifest = load_package_manifest(package, receipt['package_manifest_sha256'])
    for name, digest in manifest.items():
        if not (package/name).is_file() or sha(package/name) != digest:
            raise ValueError('Packaged source changed: ' + name)
    matrix = verify_freeze(output)
    if matrix['cases'] != read_json(root/'config/experiments/multicondition_matrix_v1.json')['cases']:
        raise ValueError('Frozen cases differ from committed matrix')
    if sha(root/'config/experiments/planar_reacquisition_v1.json') != matrix['baseline_profile_sha256']:
        raise ValueError('Original control profile changed after freeze')
    demo = _load_tool(root, 'run_functional_demo')
    environment = demo.verify_environment(root, base)
    # Include the actually loaded binary and original geometry in every attempt.
    return receipt, manifest, environment


def deployment_map(manifest):
    checks = {}
    for name, digest in manifest.items():
        if name.startswith(('landing/', 'tools/', 'config/airframes/')):
            checks[name] = digest
        elif name in ('interactive_base.py', 'operator_backend.py', 'operator_clock.py', 'operator_legacy.py'):
            checks['tools/' + name] = digest
    for name in ('operator_multicondition_backend.py', 'operator_multicondition_supervisor.py',
                 'tools/multicondition_scene.py', 'tools/multicondition_bridge.py'):
        if name not in manifest:
            raise ValueError('Missing isolated multicondition deployment source: ' + name)
    checks['tools/operator_backend.py'] = manifest['operator_multicondition_backend.py']
    return checks


def supervisor_command(package, python=None):
    return ['xvfb-run', '-a', '-s', '-screen 0 1280x800x24 -nolisten tcp',
            python or sys.executable, str(Path(package)/'operator_multicondition_supervisor.py')]


def inspect_cycle(root, output, case, row, checks):
    attempt = Path(row['attempt_dir']); run = attempt/'session_001/cycle_001'; results = run/'results'
    def optional(path):
        try: return read_json(path)
        except (OSError, ValueError): return {}
    status, closed = optional(results/'execution_status.json'), optional(results/'cycle_closed.json')
    profile = read_json(Path(output)/'profiles'/f"{case['case_id']}.json")
    integrity = bool(checks and all((run/n).is_file() and sha(run/n) == h for n,h in checks.items())
                     and optional(run/'config/experiments/receding_live_v1.json') == profile
                     and optional(results/'world_case.json') == case
                     and status.get('scenario') == case['scenario'])
    history = safe_history_check(run)
    if results.is_dir(): save(results/'request_history_audit.json', history)
    compression = None
    if (results/'model_states.jsonl').exists() or (results/'model_states.jsonl.gz').exists():
        compression = compress_raw(results)
    audit_path = results/'multicondition_audit.json'
    audit_code = None
    if results.is_dir() and not audit_path.exists():
        with (attempt/'multicondition_audit.log').open('x') as log:
            proc = subprocess.run([sys.executable, str(Path(root)/'tools/audit_multicondition.py'),
                                   str(run), '--case', str(Path(output)/'cases'/case['case_id']/'case.json')],
                                  cwd=root, stdout=log, stderr=subprocess.STDOUT)
        audit_code = proc.returncode
    audit = optional(audit_path)
    category = classify(audit, status, closed, integrity, history['passed'], row.get('supervisor_exit_code'))
    row.update(category=category, run=str(run), source_configuration_integrity=integrity,
               request_history_ok=history['passed'], native_cycle_closed=closed,
               execution_status=status, multicondition_audit=audit, audit_exit_code=audit_code,
               raw_compression=compression)
    if results.is_dir() and closed.get('all_owned_cycle_processes_stopped') is True:
        row['raw_archive'] = archive_raw(run, attempt)
    return row


def select_unattempted_cases(cases, attempted, limit=None, *, case_ids=None):
    if limit is not None and (type(limit) is not int or limit <= 0):
        raise ValueError('case-limit must be a positive integer')
    attempted_ids = set(attempted)
    declared = {case['case_id']: case for case in cases}
    if case_ids is not None:
        if not case_ids or len(set(case_ids)) != len(case_ids) or any(cid not in declared for cid in case_ids):
            raise ValueError('Targeted cases must be distinct declared IDs')
        cases = [declared[cid] for cid in case_ids]
    selected = [case for case in cases if case['case_id'] not in attempted_ids]
    return selected if limit is None else selected[:limit]


def run(root, output, base, package, source, frozen, resume=False, case_limit=None, case_ids=None):
    root, output, base, package, source, frozen = map(Path, (root, output, base, package, source, frozen))
    matrix = verify_freeze(output)
    index = read_index(output)
    authorised = read_json(output/'code_receipt.json').get('authorised_case_ids')
    if authorised is not None:
        case_ids = authorised if case_ids is None else case_ids
        if not set(case_ids) <= set(authorised):raise ValueError('Selection exceeds this authorised regression subset')
    selected = select_unattempted_cases(matrix['cases'], {v['case_id'] for v in index['attempts']}, case_limit, case_ids=case_ids)
    no_gui = os.environ.get('HS_NO_GUI', '1')
    if no_gui not in ('0', '1'):
        raise ValueError('HS_NO_GUI must be 0 or 1')
    if index['attempts'] and not resume:
        raise FileExistsError('Existing attempts retained; use --resume for only unattempted cases')
    if any(v['category'] == 'in_progress' for v in index['attempts']):
        raise RuntimeError('Unclosed prior attempt requires native recovery and evidence review')
    demo = _load_tool(root, 'run_functional_demo')
    lock = demo.acquire_lock(base)
    # Shared lock lasts the entire batch and is also inherited by each supervisor.
    with lock:
        demo.check_ports()
        receipt, manifest, environment = verify_receipt(root, output, base, package)
        if not shutil.which('xvfb-run'): raise RuntimeError('Verified Xvfb runtime is required')
        for location in (source, frozen, base/'venv/bin/python'):
            if not location.exists(): raise FileNotFoundError(str(location))
        checks = deployment_map(manifest)
        new_rows = []
        for case in selected:
            # A shared environment mutation blocks expansion before creating an attempt.
            receipt, manifest, environment = verify_receipt(root, output, base, package)
            demo.check_ports()
            row = begin_attempt(output, case['case_id'], receipt['run_commit'], control_commit=receipt['control_commit'])
            attempt = Path(row['attempt_dir']); session = attempt/'session_001'
            profile = output/'profiles'/f"{case['case_id']}.json"
            save(attempt/'declared_protocol.json', dict(run_commit=receipt['run_commit'], control_commit=receipt['control_commit'],
                 control_baseline_commit=receipt.get('control_baseline_commit', CONTROL_COMMIT),
                 control_changes=receipt.get('control_changes', {}), authorised_case_ids=authorised,
                 source_sha256=checks, environment_verified=environment, code_receipt_sha256=sha(output/'code_receipt.json'),
                 control_configuration=read_json(profile), case=case, thresholds_relaxed=False,
                 matrix_sha256=sha(output/'matrix.json'), world_only_case=True,
                 presentation_environment={'HS_NO_GUI': no_gui}, case_limit=case_limit))
            env = dict(os.environ, HS_SIM_BASE=str(base), HS_OPERATOR_SESSION=str(session),
                       HS_OPERATOR_PACKAGE=str(package), HS_LANDING_PROFILE=str(profile),
                       HS_SIM_SOURCE=str(source), HS_FROZEN_PACKAGE=str(frozen),
                       HS_MULTICONDITION_CASE=str(output/'cases'/case['case_id']/'case.json'),
                       HS_DEMO_PARTITION=uuid.uuid4().hex, HS_NO_GUI=no_gui)
            command = supervisor_command(package)
            row['command'] = command; row['environment_verified'] = environment
            row['presentation_environment'] = {'HS_NO_GUI': no_gui}
            print(json.dumps(dict(event='CASE_START',case_id=case['case_id'],run_commit=receipt['run_commit'],time=stamp())),flush=True)
            with (attempt/'supervisor.log').open('x') as log:
                child = subprocess.Popen(command, cwd=package, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True, pass_fds=(lock.fileno(),))
                # No runner timeout or kill: native supervisor owns safe LAND/disarm.
                while child.poll() is None:
                    try: ui = read_json(session/'status.json')
                    except (OSError, ValueError): ui = {}
                    if ui.get('stage') == 'ERROR':
                        request_dir = session/'requests'
                        if request_dir.is_dir() and not (attempt/'native_close_requested.json').exists():
                            request = dict(action='close')
                            save(request_dir/f'{time.time_ns()}_close.json', request)
                            save(attempt/'native_close_requested.json', dict(request=request, time=stamp()))
                        # Initialization before backend exists uses supervisor close.
                        # The adapted supervisor must independently verify native disarm.
                        (session/'close.request').touch()
                    time.sleep(.5)
            row['supervisor_exit_code'] = child.returncode
            try:
                row = inspect_cycle(root, output, case, row, checks)
            except Exception as exc:
                row.update(category='environment_or_insufficient_evidence', evidence_error=repr(exc))
                (attempt/'evidence_error.log').write_text(traceback.format_exc())
            index = finish_attempt(output, row)
            new_rows.append(row)
            print(json.dumps(dict(event='CASE_END',case_id=case['case_id'],category=row['category'],time=stamp())),flush=True)
            if row.get('native_cycle_closed', {}).get('all_owned_cycle_processes_stopped') is not True:
                write_index(output, index['attempts'], 'BLOCKED_SHARED_ENVIRONMENT_OR_SAFETY')
                return 3
            # Algorithm failures and coverage failures retain their outcome while
            # all distinct cases continue. There are no algorithm retries.
        index = read_index(output)
        stage = 'COMPLETE' if index['attempted_cases'] == 20 else 'READY_FOR_REVIEW'
        write_index(output, index['attempts'], stage)
        outcomes = new_rows if new_rows else index['attempts']
        return 0 if all(v['category'] in CATEGORIES[:2] for v in outcomes) else 2


def audit_batch(output):
    output = Path(output); matrix = verify_freeze(output)
    index = read_index(output)
    rows = []
    for case in matrix['cases']:
        attempts = [v for v in index['attempts'] if v['case_id'] == case['case_id']]
        if not attempts:
            rows.append(dict(case_id=case['case_id'], category='unattempted')); continue
        row = attempts[0]; persisted = Path(row['attempt_dir'])/'result.json'
        try: valid = persisted.is_file() and read_json(persisted) == row
        except (OSError, ValueError): valid = False
        audit = row.get('multicondition_audit', {})
        run = Path(row.get('run', Path(row['attempt_dir'])/'session_001/cycle_001'))
        valid = valid and (run/'results/multicondition_audit.json').is_file()
        try: valid = valid and read_json(run/'results/multicondition_audit.json') == audit
        except (OSError, ValueError): valid = False
        archive = row.get('raw_archive', {})
        if archive:
            valid = valid and Path(archive['path']).is_file() and sha(archive['path']) == archive['sha256']
        rows.append(dict(case_id=case['case_id'], category=row['category'] if valid else 'environment_or_insufficient_evidence',
                         persisted_evidence_intact=bool(valid), run_commit=row['run_commit']))
    report = dict(planned_cases=20, attempted_cases=len(index['attempts']), cases=rows,
                  complete=len(index['attempts']) == 20 and all(v.get('persisted_evidence_intact') for v in rows),
                  matrix_sha256=sha(output/'matrix.json'), created_utc=stamp())
    path = output/('batch_audit_' + str(time.time_ns()) + '.json')
    save(path, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare','run','audit'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--matrix', type=Path)
    for name in ('base','package','source','frozen'): parser.add_argument('--'+name,type=Path)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--case-id',action='append',help='Select declared targeted cases, in requested order')
    parser.add_argument('--case-limit',type=int,help='Run at most this many new cases, then retain READY_FOR_REVIEW')
    args = parser.parse_args(); root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if args.action == 'prepare':
        matrix = args.matrix or root/'config/experiments/multicondition_matrix_v1.json'
        prepare(root,output,read_json(matrix)); return 0
    if args.action == 'audit':
        return 0 if audit_batch(output)['complete'] else 2
    if not all(getattr(args,n) for n in ('base','package','source','frozen')):
        parser.error('run requires --base --package --source --frozen')
    return run(root,output,args.base.resolve(),args.package.resolve(),args.source.resolve(),args.frozen.resolve(),args.resume,args.case_limit,args.case_id)


if __name__ == '__main__':
    sys.exit(main())
