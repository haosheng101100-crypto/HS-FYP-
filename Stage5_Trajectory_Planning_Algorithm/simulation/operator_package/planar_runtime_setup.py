"""Portable native-desktop settings and known historical world variants."""


def desktop_environment(environment):
    defaults=dict(DISPLAY=':0',XAUTHORITY='/run/user/1000/gdm/Xauthority',XDG_RUNTIME_DIR='/run/user/1000',DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus')
    return {key:environment.get(key,default) for key,default in defaults.items()}


def prepare_world_start(text):
    old='<pose>5 0 0.002 0 0 0</pose>';current='<pose>3 0 0.002 0 0 0</pose>'
    if text.count(old)+text.count(current)!=1:
        raise ValueError('Expected exactly one original5m or archived3m initial platform pose')
    return text.replace(old,current)
