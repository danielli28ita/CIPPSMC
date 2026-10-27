#!/usr/bin/env python3
"""Compile the actual frozen trajectory/state machine against independent reference tests."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
destination = ROOT/'workspace/tests'
destination.mkdir(parents=True, exist_ok=True)
command = ['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
           '-I'+str(ROOT/'firmware/overlay/src/modules/figure8_trajectory'),
           '-I'+str(ROOT/'tests/reference/include'), str(ROOT/'tests/figure8_mission_test.cpp'),
           str(ROOT/'tests/reference/src/lemniscate_trajectory.cpp'), '-o', str(destination/'figure8_mission_test')]
subprocess.run(command, check=True)
subprocess.run([str(destination/'figure8_mission_test')], check=True)
subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT/'tests'), '-p', 'test_*.py', '-v'], check=True)
