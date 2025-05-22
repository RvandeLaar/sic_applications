#!/usr/bin/env python3
"""
Pepper-to-Pepper puppeteering
────────────────────────────
 • MASTER  : Pepper that the operator moves by hand           (ip MASTER_IP)
 • PERFORMER: Pepper that mirrors the master’s joint positions (ip PUPPET_IP)

 – Tap the master’s head   → pause / resume streaming
 – Hit <Enter> (or Ctrl-C) → finish session and put both robots to rest
"""
import threading
import time

from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoBasicAwarenessRequest,
    NaoRestRequest,
    NaoSetAutonomousLifeRequest,
    NaoWakeUpRequest,
)
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_text_to_speech import (
    NaoqiTextToSpeechRequest,
)
from sic_framework.devices.common_naoqi.pepper_motion_streamer import (
    PepperMotionStreamerConf,
    StartStreaming,
    StopStreaming,
)

# ───────────────────────────────────────
# USER-TUNABLE CONSTANTS
# ───────────────────────────────────────
MASTER_IP = "10.0.0.165"
PUPPET_IP = "10.0.0.196"
JOINTS = ["Head", "RArm", "LArm"]       # streamed chains
STREAM_HZ = 30


class PepperPuppeteer:
    """One class that owns both robots and the tactile callback."""

    def __init__(self, master_ip: str, puppet_ip: str):
        # Motion-streamer configurations
        master_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ)
        puppet_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ,
                                               stiffness=1)  # performer fully stiff

        self.master = Pepper(master_ip, pepper_motion_conf=master_conf)
        self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf)

        self._paused = False
        self._stop_flag = threading.Event()   # set() when operator presses <Enter>

        self._setup_robots()

        # Head-touch toggles pause/resume
        self.master.tactile_sensor.register_callback(self._on_head_touch)

    # ────────────────────────────────
    # Low-level helpers
    # ────────────────────────────────
    def _say(self, text: str):
        self.master.tts.request(
            NaoqiTextToSpeechRequest(text, language="English")
        )

    def _start_streaming(self):
        self.master.motion_streaming.request(StartStreaming(JOINTS))

    def _stop_streaming(self):
        self.master.motion_streaming.request(StopStreaming())

    # ────────────────────────────────
    # One-off robot initialisation
    # ────────────────────────────────
    def _setup_robots(self):
        print("\n🔧  Initialising robots …")

        # Disable life/autonomous abilities that fight against manual puppeteering
        self.master.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))
        self.puppet.autonomous.request(NaoBasicAwarenessRequest(False))

        # Wake up (Pepper cannot change stiffness while in rest)
        for bot in (self.master, self.puppet):
            bot.autonomous.request(NaoWakeUpRequest())

        # Master: zero stiffness on streamed chains so the operator can move it freely
        self.master.stiffness.request(
            Stiffness(0.0, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Connect the performer’s motion-stream listener to the master’s streamer
        self.puppet.motion_streaming.connect(self.master.motion_streaming)

        print("✅  Robots are ready – tap the head to pause/resume.")

    # ────────────────────────────────
    # Tactile-sensor callback
    # ────────────────────────────────
    def _on_head_touch(self, _msg):
        if self._paused:
            self._say("Resuming puppeteering")
            self._start_streaming()
            self._paused = False
        else:
            self._say("Pausing puppeteering")
            self._stop_streaming()
            self._paused = True

    # ────────────────────────────────
    # Main user-facing entry point
    # ────────────────────────────────
    def run(self):
        print("\n🎮  Starting puppeteering – press <Enter> to stop.")
        self._say("Start puppeteering")
        self._start_streaming()

        try:
            # Wait here until the operator presses Enter or hits Ctrl-C
            input()
            print("⏹  Stop requested by operator.")
        except KeyboardInterrupt:
            print("\n⏹  Ctrl-C received – stopping.")
        finally:
            self._shutdown()

    # ────────────────────────────────
    # Graceful shutdown (clean-up)
    # ────────────────────────────────
    def _shutdown(self):
        print("\n🔻  Cleaning up – please wait …")
        self._say("We are done puppeteering")

        # Make sure streaming is off and master is stiff again
        self._stop_streaming()
        self.master.stiffness.request(
            Stiffness(0.7, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Re-enable autonomous life so Pepper regains obstacle avoidance etc.
        self.master.autonomous.request(NaoSetAutonomousLifeRequest("solitary"))

        # Put both robots in rest
        for bot in (self.master, self.puppet):
            bot.autonomous.request(NaoRestRequest())

        print("Both Peppers are in rest pose. Shutting down.")


# ───────────────────────────────────────
# Script entry point
# ───────────────────────────────────────
if __name__ == "__main__":
    PepperPuppeteer(master_ip=MASTER_IP, puppet_ip=PUPPET_IP).run()