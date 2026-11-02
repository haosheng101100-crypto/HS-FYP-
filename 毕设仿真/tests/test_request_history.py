"""Each published planner request must remain recoverable after cancellation."""
from pathlib import Path
import json,tempfile,unittest
from landing.receding_live import FilePlannerClient
from landing.receding_landing import FlightState,PlatformState

class RequestHistoryTests(unittest.TestCase):
 def test_cancelled_request_survives_next_submission_exactly(self):
  with tempfile.TemporaryDirectory() as tmp:
   client=FilePlannerClient(tmp)
   state=FlightState(1.,(0.,0.,4.),(0.,0.,0.),(0.,0.,0.))
   platform=PlatformState(1.,(2.,0.,.8),(.25,0.,0.),(0.,0.,0.))
   first=client.submit(state,platform,(6.,8.,10.),None,tracking_envelope={'test':{'reserve':.02}})
   original=(Path(tmp)/'request.json').read_bytes()
   client.abandon()
   second=client.submit(state,platform,(8.,10.),12.)
   archived=Path(tmp)/'request_000001.json'
   self.assertTrue(archived.is_file(),'Cancelled original request was overwritten instead of retained')
   self.assertEqual(json.loads(archived.read_text()),json.loads(original))
   self.assertEqual(json.loads((Path(tmp)/'request_000002.json').read_text()),json.loads((Path(tmp)/'request.json').read_text()))
   self.assertEqual((first['sequence'],second['sequence']),(1,2))
 def test_reused_directory_does_not_overwrite_archived_sequence(self):
  with tempfile.TemporaryDirectory() as tmp:
   path=Path(tmp)/'request_000001.json';path.write_text('{"preserve":true}')
   client=FilePlannerClient(tmp)
   state=FlightState(1.,(0.,0.,4.),(0.,0.,0.),(0.,0.,0.))
   platform=PlatformState(1.,(2.,0.,.8),(.25,0.,0.),(0.,0.,0.))
   with self.assertRaises(FileExistsError):client.submit(state,platform,(6.,8.,10.),None)
   self.assertEqual(path.read_text(),'{"preserve":true}')
   self.assertFalse((Path(tmp)/'request.json').exists())
if __name__=='__main__':unittest.main()
