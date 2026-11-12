import json
import tempfile
import unittest
from pathlib import Path
from jetbot_semantic_nav.core import CaseEventReader


class CaseAcknowledgementTest(unittest.TestCase):
    def test_skip_old_task_and_wait_for_complete_utf8_event(self):
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'events.jsonl';path.write_text('{"task_id":"same","state":"SUCCEEDED"}\n')
            reader=CaseEventReader(path)
            try:
                self.assertEqual(reader.read(),[])
                row=dict(task_id='same',state='DUPLICATE',message='纸箱')
                data=json.dumps(row,ensure_ascii=False).encode()+b'\n'
                cut=data.index('纸'.encode())+1
                with path.open('ab')as stream:stream.write(data[:cut])
                self.assertEqual(reader.read(),[])
                with path.open('ab')as stream:stream.write(data[cut:])
                self.assertEqual(reader.read(),[row]);self.assertEqual(reader.read(),[])
            finally:reader.close()
