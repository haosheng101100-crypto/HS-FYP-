"""Disarmed is not synonymous with a normal landing."""
def classify_disarm(armed_at_save,events,final_mode,arm_records=()):
    texts=[str(e.get('message','') if isinstance(e,dict) else e) for e in events]
    crash=[t for t in texts if 'crash:' in t.lower() or 'crash detected' in t.lower()]
    forced=any(r.get('ArmState')==0 and bool(r.get('Forced')) for r in arm_records)
    stopped=not armed_at_save
    normal=stopped and not crash and not forced and final_mode=='LAND'
    reason='STILL_ARMED' if not stopped else 'CRASH_AUTODISARM' if crash else 'FORCED_DISARM' if forced else 'NORMAL_LAND_DISARM' if normal else 'DISARMED_REASON_UNVERIFIED'
    return {'disarmed_observed':stopped,'normal_disarm_observed':normal,'crash_detected':bool(crash),
            'crash_messages':crash,'forced_disarm_detected':forced,'disarm_classification':reason}
