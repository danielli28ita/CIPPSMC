#!/usr/bin/env python3
"""Offline consistency checks for the compact research repository."""

import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class RepositoryTests(unittest.TestCase):
    def test_overlay_manifest(self):
        overlay = ROOT / "firmware"
        metadata = json.loads((overlay / "BASELINE.json").read_text())
        self.assertEqual(metadata["baseline_commit"], "99c40407ffd7ac184e2d7b4b293f36f10fe561ef")
        self.assertEqual(metadata["controller_selector"], 0)
        self.assertEqual(len(metadata["files"]), 9)

        for relative, expected in metadata["files"].items():
            payload = overlay / "changed" / relative
            self.assertTrue(payload.is_file(), relative)
            content = payload.read_bytes()
            self.assertEqual(len(content), expected["bytes"], relative)
            self.assertEqual(hashlib.sha256(content).hexdigest(), expected["sha256"], relative)

    def test_controller_selector(self):
        source = (ROOT / "firmware/changed/src/modules/mc_pos_control/PositionControl/PositionControl.cpp").read_text()
        self.assertIn("#define BASELINE_CONTROLLER 0", source)
        self.assertIn("_applyBppsmcOverride(acc_sp_pid)", source)

    def test_no_local_absolute_paths(self):
        forbidden = ("/" + "home/", "/" + "mnt/hgfs/", "C:" + "\\Users\\")
        for path in ROOT.rglob("*"):
            if path.is_file() and ".git" not in path.parts:
                try:
                    text = path.read_text()
                except UnicodeDecodeError:
                    continue
                for marker in forbidden:
                    self.assertNotIn(marker, text, str(path.relative_to(ROOT)))


if __name__ == "__main__":
    unittest.main()
