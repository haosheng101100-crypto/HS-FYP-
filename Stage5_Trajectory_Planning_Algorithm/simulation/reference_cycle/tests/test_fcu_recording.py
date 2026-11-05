from pathlib import Path
import json,tempfile,unittest
from landing.fcu_recording import FcuDiagnosticCapture

class Message:
    def __init__(self,kind='ATTITUDE'):self.kind=kind
    def get_type(self):return self.kind
    def get_srcSystem(self):return 1
    def get_srcComponent(self):return 1
    def to_dict(self):return {'mavpackettype':self.kind,'time_boot_ms':1234,'pitch':float('nan'),'q':[1.,0.,0.,0.]}

class FcuRecordingChecks(unittest.TestCase):
    def test_native_clock_retained_and_receipt_hint_separate(self):
        capture=FcuDiagnosticCapture();capture.record(Message(),100.,1.3)
        row=capture.rows[0];self.assertEqual(row['message']['time_boot_ms'],1234)
        self.assertEqual(row['gazebo_sim_t_at_receipt_hint_s'],1.3)
        self.assertEqual(row['received_monotonic_s'],100.)
    def test_nonfinite_fields_and_absent_stream_are_explicit(self):
        capture=FcuDiagnosticCapture();capture.record(Message(),100.,None)
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as folder:
            root=Path(folder);capture.save(root)
            row=json.loads((root/'fcu_diagnostic_messages.jsonl').read_text())
            summary=json.loads((root/'fcu_diagnostic_capture.json').read_text())
        self.assertIsNone(row['message']['pitch']);self.assertIn('POSITION_TARGET_LOCAL_NED',summary['missing_types'])
        self.assertEqual(summary['rows'],1)
    def test_ignored_message_not_invented_and_invalid_clock_rejected(self):
        capture=FcuDiagnosticCapture();capture.record(Message('HEARTBEAT'),100.,1.)
        self.assertEqual(capture.rows,[])
        with self.assertRaises(ValueError):capture.record(Message(),float('nan'),1.)
