#!/usr/bin/env python3
"""Run the onboard task in local SITL with a simulated RC switch. Publish no trajectory setpoint."""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT / "workspace/PX4-Autopilot"


class TestRun:
    def __init__(self, destination, sim_hover, extra_env=None):
        self.destination = destination
        self.rootfs = destination / "rootfs"
        self.rootfs.mkdir(parents=True)
        self.log_path = destination / "px4.log"
        self.log = self.log_path.open("w")
        env = os.environ.copy()
        env["BPP_REPRO_PROFILE"] = str(ROOT / "scripts/sitl_profile.rc")
        env["BPP_SIM_HOVER"] = str(sim_hover)
        env.update(PX4_SYS_AUTOSTART="4001", PX4_SIM_MODEL="gz_x500", HEADLESS="1",
                   PX4_GZ_SIM_RENDER_ENGINE="ogre", PX4_GZ_WORLD="default",
                   PX4_GZ_STANDALONE="1", GZ_PARTITION="figure8_" + str(time.time_ns()),
                   PX4_GZ_WORLDS=str(PX4 / "Tools/simulation/gz/worlds"),
                   PX4_GZ_MODELS=str(PX4 / "Tools/simulation/gz/models"),
                   GZ_SIM_RESOURCE_PATH=str(PX4 / "Tools/simulation/gz/models") + ":"
                   + str(PX4 / "Tools/simulation/gz/worlds"))
        if extra_env:
            env.update(extra_env)
        self.gz_log = (destination / "gazebo.log").open("w")
        self.gz_process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "-v", "3", "--render-engine", "ogre",
             str(PX4 / "Tools/simulation/gz/worlds/default.sdf")], env=env,
            stdout=self.gz_log, stderr=subprocess.STDOUT, start_new_session=True)
        print("Waiting for the Gazebo model spawn service...", flush=True)
        deadline = time.monotonic() + 90
        ready = False
        while time.monotonic() < deadline:
            if self.gz_process.poll() is not None:
                break
            try:
                services = subprocess.run(["gz", "service", "-l"], env=env, capture_output=True,
                                          text=True, timeout=5)
                if "/world/default/create" in services.stdout:
                    ready = True
                    break
            except subprocess.TimeoutExpired:
                pass
        if not ready:
            self.gz_process.terminate()
            self.gz_process.wait(timeout=10)
            self.gz_log.close()
            raise RuntimeError("Gazebo service was not ready; see gazebo.log")
        self.process = subprocess.Popen(
            [str(PX4 / "build/px4_sitl_default/bin/px4"),
             str(PX4 / "build/px4_sitl_default/etc"), "-s", str(Path(__file__).with_name("sitl_onboard.rc"))],
            cwd=self.rootfs, env=env, stdin=subprocess.PIPE,
            stdout=self.log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        self.link = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255,
                                             source_component=190, dialect="common")
        self.channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1500]
        self.rc_enabled = True
        self.latest = {}
        self.rows = []
        self.last_sent = 0
        self.last_heartbeat = 0

    def command(self, text):
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()

    def pump(self, duration=0.1):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            if self.process.poll() is not None:
                raise RuntimeError("PX4 exited; see " + str(self.log_path))
            now = time.monotonic()
            if now - self.last_heartbeat >= 0.5:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.last_heartbeat = now
            if self.rc_enabled and now - self.last_sent >= 0.04:
                self.link.mav.rc_channels_override_send(1, 1, *self.channels)
                self.last_sent = now
            for _ in range(300):
                message = self.link.recv_match(blocking=False)
                if message is None:
                    break
                self.latest[message.get_type()] = message
                if message.get_type() == "STATUSTEXT":
                    print(message.text, flush=True)
                if message.get_type() == "LOCAL_POSITION_NED":
                    self.rows.append(dict(t=now, x=message.x, y=message.y, z=message.z,
                                          vx=message.vx, vy=message.vy, vz=message.vz))
            time.sleep(0.01)

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            if predicate():
                return
        raise RuntimeError("Timeout: " + label)

    def text(self):
        return self.log_path.read_text(errors="replace")

    def armed(self):
        heartbeat = self.latest.get("HEARTBEAT")
        return heartbeat is not None and bool(heartbeat.base_mode & 128)

    def mode(self):
        heartbeat = self.latest.get("HEARTBEAT")
        return ((heartbeat.custom_mode >> 16) & 255) if heartbeat else -1

    def start_mission(self, controller):
        self.channels[5] = 1000
        self.pump(0.5)
        self.channels[4] = 1000
        self.pump(0.5)
        self.channels[2] = 1000
        self.command("param set MPC_CTRL_METHOD " + str(controller))
        self.pump(3)
        self.command("mc_pos_control status")
        offset = len(self.text())
        self.command("figure8_trajectory status")
        self.wait(lambda: "stage: ready" in self.text()[offset:], 30, "module ready")
        self.channels[5] = 2000
        self.wait(self.armed, 15, "RC arming")
        self.channels[2] = 1500
        self.pump(1)
        self.channels[4] = 2000
        self.wait(lambda: self.mode() == 6, 10, "RC OFFBOARD selection")

    def full_flight(self, controller):
        offset = len(self.text())
        start_row = len(self.rows)
        self.start_mission(controller)
        self.wait(lambda: not self.armed(), 180, "full flight and landing/disarm")
        self.pump(1)
        text = self.text()[offset:]
        assert "Figure8 started: " + ("BPPSMC" if controller else "PID") in text
        assert "Figure8: figure8" in text and "Figure8: landing" in text
        assert "Figure8 complete" in text and "Figure8 aborted" not in text
        rows = self.rows[start_row:]
        result = {"method": controller, "complete": True,
                  "max_relative_estimated_height": max(-row["z"] for row in rows) + rows[0]["z"]}
        self.channels[5] = 1000
        self.pump(0.5)
        self.channels[4] = 1000
        self.pump(0.5)
        self.channels[2] = 1000
        self.pump(3)
        return result

    def takeover(self):
        self.start_mission(1)
        self.pump(16)
        offset = len(self.text())
        self.channels[4] = 1000
        self.wait(lambda: self.mode() == 3, 5, "Position takeover")
        self.pump(2)
        self.command("listener bppsmc_debug -n 1")
        self.command("mc_pos_control status")
        self.command("figure8_trajectory status")
        self.pump(1)
        assert "stage: aborted" in self.text()[offset:]
        assert "Onboard policy: native PID takeover" in self.text()[offset:]
        self.channels[4] = 2000
        self.pump(3)
        assert "Figure8 started" not in self.text()[offset:]
        self.channels[4] = 1000
        self.pump(1)
        self.command("commander land")
        self.wait(lambda: not self.armed(), 60, "takeover landing")
        self.channels[5] = 1000
        self.channels[2] = 1000
        self.pump(3)
        return {"position_takeover": True, "no_airborne_restart": True}

    def kill_and_link_loss(self):
        self.start_mission(1)
        self.pump(1)
        offset = len(self.text())
        self.channels[6] = 2000
        self.pump(2)
        self.command("figure8_trajectory status")
        self.command("listener actuator_armed -n 1")
        self.pump(1)
        assert "stage: aborted" in self.text()[offset:]
        assert "manual_lockdown: True" in self.text()[offset:] or "manual_lockdown: true" in self.text()[offset:]
        self.channels[5] = 1000
        self.pump(0.5)
        self.channels[4] = 1000
        self.wait(lambda: not self.armed(), 15, "Kill disarm")
        self.channels[6] = 1000
        self.channels[2] = 1000
        self.pump(3)
        self.start_mission(1)
        self.pump(16)
        offset = len(self.text())
        self.rc_enabled = False
        self.pump(2)
        self.command("figure8_trajectory status")
        self.pump(1)
        assert "stage: aborted" in self.text()[offset:]
        self.rc_enabled = True
        self.channels[4] = 1000
        self.pump(2)
        self.command("commander land")
        self.wait(lambda: not self.armed(), 70, "RC loss recovery landing")
        return {"kill_aborts_task": True, "RC_loss_aborts_task": True}

    def close(self):
        self.channels[5] = 1000
        self.channels[4] = 1000
        try:
            self.command("shutdown")
            self.process.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        try:
            os.killpg(self.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        self.link.close()
        self.log.close()
        if self.gz_process.poll() is None:
            self.gz_process.terminate()
            try:
                self.gz_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.gz_process.kill()
                self.gz_process.wait()
        self.gz_log.close()
        (self.destination / "telemetry.json").write_text(json.dumps(self.rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-sitl", action="store_true", help="Explicitly authorize local simulator launch and simulated RC arming")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--controller", choices=["bppsmc", "pid"], default="bppsmc")
    parser.add_argument("--sim-hover", type=float, default=0.73, help="Simulator-only X500 mapping, never an aircraft parameter recommendation")
    parser.add_argument("--extra-checks", action="store_true", help="Also run takeover, Kill and RC loss tests")
    args = parser.parse_args()
    if not args.execute_sitl:
        parser.error("Pass --execute-sitl to launch and arm only the local simulator")
    if not 0.1 <= args.sim_hover <= 0.9:
        parser.error("--sim-hover must be between 0.1 and 0.9")
    if subprocess.run(["pgrep", "-x", "px4"], capture_output=True).returncode == 0:
        parser.error("An existing PX4 process is running; stop it before this isolated test")
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("127.0.0.1", 18570))
        except OSError:
            parser.error("UDP 18570 is occupied; no existing process will be stopped")
    output = args.output or ROOT / "workspace/results" / datetime.datetime.now().strftime("sitl_%Y%m%d_%H%M%S")
    if output.exists():
        parser.error("Output already exists; choose a new directory")
    run = TestRun(output.resolve(), args.sim_hover)
    result = {"hardware": False, "setpoints_from_host": False, "RC_input": "MAVLink RC_CHANNELS_OVERRIDE",
              "trajectory": "Bernoulli lemniscate with quintic phase", "simulator_only_hover_mapping": args.sim_hover}
    try:
        run.wait(lambda: "Startup script returned successfully" in run.text(), 90, "startup")
        run.wait(lambda: "Figure8 ready" in run.text(), 90, "estimator and task ready")
        result["flights"] = [run.full_flight(1 if args.controller == "bppsmc" else 0)]
        if args.extra_checks:
            result["takeover"] = run.takeover()
            result["interruptions"] = run.kill_and_link_loss()
        result["passed"] = True
    except BaseException as error:
        result["passed"] = False
        result["error"] = str(error)
        raise
    finally:
        run.close()
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        print("Result directory:", output, flush=True)


if __name__ == "__main__":
    main()
