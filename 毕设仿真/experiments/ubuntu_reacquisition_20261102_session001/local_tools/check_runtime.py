from pathlib import Path
import hashlib, json, os, platform, shutil, subprocess, sys

workspace = Path('/workspace')
repo = workspace / 'work/reacquisition_20261102/repository/毕设仿真'
base = workspace / 'work/reacquisition_20261102/deployment'
vendor = Path('/opt/hs-vendor')
source = json.loads((repo / 'simulation/environment_reference/source_snapshot.json').read_text())
plugin = vendor / 'ardupilot_gazebo'
assert subprocess.check_output(['git','-C',str(plugin),'rev-parse','HEAD'],text=True).strip() == source['plugin_commit']
for name, digest in source['files'].items():
    assert hashlib.sha256((plugin/name).read_bytes()).hexdigest() == digest, name
binary = workspace/'work/simulation_environment/arducopter_4.7.1'
assert hashlib.sha256(binary.read_bytes()).hexdigest() == source['arducopter_sha256']
shutil.copyfile(binary, vendor/'arducopter')
(vendor/'arducopter').chmod(0o755)
for p in Path('/usr/share/gz').glob('*.yaml'):
    shutil.copyfile(p, base/'runtime/gz_config'/p.name)
assert (base/'gz_prefix/usr/share/gz/gz-sim8/gui/gui.config').is_file()
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gi.repository import GLib
import cvxpy, clarabel, numpy, scipy
probe=base/'environment_probe'
(probe/'scripts').mkdir(parents=True,exist_ok=True)
shutil.copyfile(base/'reference/scripts/env.sh',probe/'scripts/env.sh')
for name in ('venv','vendor','gz_prefix'):
    if not (probe/name).is_symlink():(probe/name).symlink_to(base/name,target_is_directory=True)
code="import json,numpy,scipy,cvxpy,clarabel; print(json.dumps({n:{'version':m.__version__,'file':m.__file__} for n,m in [('numpy',numpy),('scipy',scipy),('cvxpy',cvxpy),('clarabel',clarabel)]}))"
probe_output=subprocess.check_output(['bash','-c','source "$1"; exec "$2" -c "$3"','probe',str(probe/'scripts/env.sh'),'/opt/hs-venv/bin/python',code],text=True)
runtime_modules=json.loads(probe_output)
for n,m in [('numpy',numpy),('scipy',scipy),('cvxpy',cvxpy),('clarabel',clarabel)]:
    assert runtime_modules[n]['version']==m.__version__, ('Actual child environment overrides pinned package',runtime_modules)
sys.path.insert(0, str(base/'reference'))
from landing.reference_loader import load_verified_reference
load_verified_reference(base/'reference')
receipt = dict(os=Path('/etc/os-release').read_text(), python=platform.python_version(),
    plugin_commit=source['plugin_commit'], plugin_source_checks_passed=True,
    arducopter_sha256=source['arducopter_sha256'],
    package_versions={n: m.__version__ for n,m in [('numpy',numpy),('scipy',scipy),('cvxpy',cvxpy),('clarabel',clarabel)]},
    gazebo=subprocess.check_output(['gz','sim','--versions'],text=True).strip(),
    modules_under_actual_env_sh=runtime_modules,
    display=os.environ.get('DISPLAY'), xauthority_present=Path(os.environ['XAUTHORITY']).is_file(),
    original_geometry_reference_preflight=True)
(workspace/'outputs/重新跟随专项_20261102/environment_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2))
print(json.dumps(receipt,ensure_ascii=False,indent=2))
