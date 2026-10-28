"""Active wall time: deliberate physics pauses stop watchdog timeout clocks."""
import threading,time

class OperatorClock:
    def __init__(self,source=time.monotonic):
        self.source=source;self.lock=threading.RLock();self.total=0.;self.since=None
    @property
    def paused(self):
        with self.lock:return self.since is not None
    def monotonic(self):
        with self.lock:return (self.source() if self.since is None else self.since)-self.total
    def set_paused(self,paused):
        with self.lock:
            now=self.source()
            if paused and self.since is None:self.since=now
            elif not paused and self.since is not None:
                self.total+=now-self.since;self.since=None

def allowed_actions(stage,armed,paused):
    return {'takeoff':stage=='READY' and not armed,'land':stage=='HOVER' and armed,
        'pause':not paused,'resume':paused,'reset':not armed and stage in ('READY','COMPLETE','ERROR')}
