"""Check provenance, parameter export round trips and compiled-controller payload identity."""
import hashlib
import json
from pathlib import Path
import unittest
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]


class ReleaseChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = list((ROOT/'data').glob('*.ulg'))
        cls.log = ULog(str(cls.samples[0]))

    def test_only_requested_sample(self):
        self.assertEqual([p.name for p in self.samples], ['log_19_2026-10-19-10-01-54.ulg'])
        self.assertEqual(hashlib.sha256(self.samples[0].read_bytes()).hexdigest(), '34d4087e9f9f53c8364c87fd2501edbf2f88131b98b14af4910c027afb3eea7e')

    def test_parameters_exactly_from_log(self):
        self.assertEqual(json.loads((ROOT/'params/all_parameters.json').read_text()), self.log.initial_parameters)
        self.assertEqual(self.log.changed_parameters, [])
        exported = {}
        for line in (ROOT/'params/all_parameters_REFERENCE_ONLY.params').read_text().splitlines():
            if line.startswith('#'):
                continue
            _, _, key, value, kind = line.split()
            exported[key] = int(value) if kind == '6' else float(value)
        self.assertEqual(exported, self.log.initial_parameters)

    def test_task_profile_not_calibration(self):
        profile = json.loads((ROOT/'params/flight_profile.json').read_text())
        for key, value in profile.items():
            self.assertEqual(value, self.log.initial_parameters[key])
            self.assertTrue(key.startswith(('BPP_', 'F8_')) or key in ('MPC_THR_HOVER', 'MPC_CTRL_METHOD'))

    def test_every_overlay_hash(self):
        meta = json.loads((ROOT/'firmware/BASELINE.json').read_text())
        for name, entry in meta['files'].items():
            self.assertEqual(hashlib.sha256((ROOT/'firmware/overlay'/name).read_bytes()).hexdigest(), entry['sha256'])

    def test_sample_completed(self):
        self.assertTrue(any('Figure8 complete: landed and disarmed' in m.message for m in self.log.logged_messages))
        self.assertEqual(self.log.initial_parameters['MPC_CTRL_METHOD'], 1)
        self.assertEqual(self.log.initial_parameters['BPP_KS_Z'], 2.)


if __name__ == '__main__':
    unittest.main()
