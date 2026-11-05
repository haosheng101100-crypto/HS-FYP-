"""Optional presentation processes; native physics/control remain separate."""
import os,subprocess
class DisabledViewer:
 pid=None
 def poll(self):return None
 def terminate(self):pass

def launch_viewer(command,env=None,**kwargs):
 environment=os.environ if env is None else env
 flag=environment.get('HS_NO_GUI','0')
 if flag not in ('0','1'):raise ValueError('HS_NO_GUI must be0 or1')
 return DisabledViewer() if flag=='1' else subprocess.Popen(command,env=env,**kwargs)
