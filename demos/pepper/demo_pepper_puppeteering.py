#!/usr/bin/env python3
"""
Pepper-to-Pepper puppeteering demo
---------------------------------
  • PUPPET      : Pepper moved manually by the operator.
  • PERFORMER   : Pepper that mirrors the puppet's joint positions.

Controls
--------
  • Tap the puppet's top-tactile sensor to pause or resume streaming.
  • Press <Enter> (or Ctrl-C) in the terminal to finish the session.

Both robots are always returned to a safe rest pose when the program exits.
"""

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

# ---------------------------------------------------------------------------
# User-configured constants
# ---------------------------------------------------------------------------
PUPPET_IP = "10.0.0.168"
PERFORMER_IP = "10.0.0.196"
JOINTS = ["Head", "RArm", "LArm"]            # motion chains to stream
STREAM_HZ = 30                               # samples per second


class PepperPuppeteer:
    """Encapsulates the two robots and all puppeteering logic."""

    def __init__(self, puppet_ip: str, performer_ip: str):
        # Motion-streamer configurations
        puppet_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ)
        performer_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=1.0)   # performer fully stiff

        self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf)
        self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf)
        self._paused = False

        self._initialise_robots()

        # Head-touch toggles pause/resume
        self.puppet.tactile_sensor.register_callback(self._on_head_touch)

    # ---------------------------------------------------------------------
    # Initialisation helpers
    # ---------------------------------------------------------------------
    def _initialise_robots(self):
        # Disable autonomous behaviour that interferes with manual control
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))
        self.performer.autonomous.request(NaoBasicAwarenessRequest(False))

        # Wake up (Pepper cannot change stiffness while in rest)
        self.puppet.autonomous.request(NaoWakeUpRequest())
        self.performer.autonomous.request(NaoWakeUpRequest())

        # Puppet: zero stiffness on streamed chains so the operator can move joints
        self.puppet.stiffness.request(
            Stiffness(0.2, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Connect performer’s motion-stream input to the master’s output
        self.performer.motion_streaming.connect(self.puppet.motion_streaming)

    # ---------------------------------------------------------------------
    # Convenience wrappers
    # ---------------------------------------------------------------------
    def _say(self, text: str) -> None:
        self.puppet.tts.request(
            NaoqiTextToSpeechRequest(text, language="English")
        )

    def _start_streaming(self) -> None:
        self.puppet.motion_streaming.request(StartStreaming(JOINTS))

    def _stop_streaming(self) -> None:
        self.puppet.motion_streaming.request(StopStreaming(JOINTS))

    # ---------------------------------------------------------------------
    # Tactile-sensor callback
    # ---------------------------------------------------------------------
    def _on_head_touch(self, _msg) -> None:
        if self._paused:
            self._say("Resuming puppeteering.")
            self._start_streaming()
            self.performer.autonomous.request(
            NaoSetAutonomousLifeRequest("disabled")   # disable autonomous life to teleoperate performer
        )
            self.performer.autonomous.request(NaoBasicAwarenessRequest(False))
            self._paused = False
        else:
            self._say("Pausing puppeteering.")
            self._stop_streaming()
            self.performer.autonomous.request(
            NaoSetAutonomousLifeRequest("solitary")   # or "interactive", put performer in autonomous mode while puppet stops streaming motions
        )
            self.performer.autonomous.request(NaoBasicAwarenessRequest(True))
            self._paused = True

    # ---------------------------------------------------------------------
    # Public entry point
    # ---------------------------------------------------------------------
    def run(self) -> None:
        print("Starting puppeteering session. Press <Enter> to stop.")
        self._say("Start puppeteering.")
        self._start_streaming()

        try:
            input()  # Blocks until the operator presses Enter
        except KeyboardInterrupt:
            print("\nInterrupt received. Stopping session.")
        finally:
            self._shutdown()

    # ---------------------------------------------------------------------
    # Clean-up
    # ---------------------------------------------------------------------
    def _shutdown(self) -> None:
        self._say("We are done puppeteering.")
        self._stop_streaming()

        # Restore puppet stiffness before rest
        self.puppet.stiffness.request(
            Stiffness(0.7, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Re-enable autonomous life (optional but recommended for Pepper)
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("solitary"))

        # Put both robots into rest
        self.puppet.autonomous.request(NaoRestRequest())
        self.performer.autonomous.request(NaoRestRequest())

        print("Session ended; both Peppers are in rest pose.")


# ---------------------------------------------------------------------------
# Script entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    PepperPuppeteer(puppet_ip=PUPPET_IP, performer_ip=PERFORMER_IP).run()