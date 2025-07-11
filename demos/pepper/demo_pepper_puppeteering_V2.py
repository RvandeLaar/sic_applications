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

import os
import threading
import time

# -------------------------------------------------------------------------------------------------------------------------
# Developer constants: if you run this script in dev_test mode, you have to provide the Redis password and your laptop's IP
# -------------------------------------------------------------------------------------------------------------------------
# Set Redis password environment variable
os.environ['REDIS_PASSWORD'] = 'changemeplease' # Do NOT change actually change it
# Set Redis IP for robots to connect to your laptop
os.environ['DB_IP'] = '10.0.0.198'  # Your Mac's IP address

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

from sic_framework.devices.common_naoqi.naoqi_motion import (
    NaoqiMoveTowardRequest,
    NaoqiGetRobotVelocityRequest,
    NaoqiCollisionProtectionRequest,
)

# ---------------------------------------------------------------------------
# User-configured constants
# ---------------------------------------------------------------------------
PUPPET_IP = "10.0.0.168"
PERFORMER_IP = "10.0.0.152"
JOINTS = ["Head", "RArm", "LArm"]            # motion chains to stream
STREAM_HZ = 30                               # samples per second
MOVE_SPEED = 0.4                             # 0 … 1  (≈10–12 cm/s on Pepper)
VEL_THR = 0.02                               # m/s, below this we treat the robot as "still"
CHECK_DT = 0.2                               # seconds, how often to check velocity


class PepperPuppeteer:
    """Encapsulates the two robots and all puppeteering logic."""

    def __init__(self, puppet_ip: str, performer_ip: str):
        # Motion-streamer configurations
        puppet_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ)
        performer_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=1.0)   # performer fully stiff

        # Use dev_test mode to test new implementations
        # self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf, dev_test=True, test_repo="/Users/rvandelaar/Desktop/School/Master AI Thesis/MSc_AI_Thesis/social-interaction-cloud")
        # self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf, dev_test=True, test_repo="/Users/rvandelaar/Desktop/School/Master AI Thesis/MSc_AI_Thesis/social-interaction-cloud")

        # Once installed, you can switch to:
        self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf, dev_test=True)
        self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf, dev_test=True)

        # State variables
        self._walking = False       # True if the performer is walking
        self._paused = False        # True if the puppeteering is paused
        self._prot_off = False      # True if collision protection is off
        self._vel_mon_stop = threading.Event()  # Event to stop velocity monitoring
        self._vel_mon_thr = None                # Thread to monitor velocity

        # Initialise robots
        self._initialise_robots()

        # Head-touch toggles pause/resume
        self.puppet.tactile_sensor.register_callback(self._on_head_touch)

        # Back bumper toggles walking/stopping
        self.puppet.back_bumper.register_callback(self._on_back_bumper)

        # Threads
        self._start_velocity_monitor()

    # ---------------------------------------------------------------------
    # Initialisation
    # ---------------------------------------------------------------------
    def _initialise_robots(self):
        # Disable autonomous behaviour that interferes with manual control
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))
        self.performer.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))

        # self.performer.autonomous.request(NaoBasicAwarenessRequest(False)

        # Wake up (Pepper cannot change stiffness while in rest)
        self.puppet.autonomous.request(NaoWakeUpRequest())
        self.performer.autonomous.request(NaoWakeUpRequest())

        # Puppet: zero stiffness on streamed chains so the operator can move joints
        self.puppet.stiffness.request(
            Stiffness(0, joints=JOINTS, enable_joint_list_generation=False)
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
            self._paused = False
        else:
            self._say("Pausing puppeteering.")
            self._stop_streaming()
            self._paused = True

    # ---------------------------------------------------------------------
    # Back bumper callback
    # ---------------------------------------------------------------------
    def _on_back_bumper(self, msg) -> None:
        if msg.value != 1:      # ignore release
            return

        if self._walking:       # → STOP
            try:
                # Stop walking
                self.performer.motion.request(NaoqiMoveTowardRequest(0.0, 0.0, 0.0))
                # Resume streaming after stopping
                self._start_streaming()
            except Exception as e:
                print("Error sending stop:", e)
            else:
                self._say("Stopping.")
                self._walking = False

        else:                   # → WALK
            try:
                # Stop streaming before walking because otherwise the performer will not move forward
                self._stop_streaming()
                self.performer.motion.request(NaoqiMoveTowardRequest(MOVE_SPEED, 0.0, 0.0))
            except Exception as e:
                print("Error sending walk command:", e)
            else:
                self._say("Walking.")
                self._walking = True

    # ---------------------------------------------------------------------
    # Velocity monitoring
    # ---------------------------------------------------------------------
    def _start_velocity_monitor(self):
        if self._vel_mon_thr and self._vel_mon_thr.is_alive():
            return
        self._vel_mon_stop.clear()
        self._vel_mon_thr = threading.Thread(
            target=self._velocity_loop, daemon=True
        )
        self._vel_mon_thr.start()

    def _stop_velocity_monitor(self):
        self._vel_mon_stop.set()
        if self._vel_mon_thr:
            self._vel_mon_thr.join()

    def _velocity_loop(self):
        while not self._vel_mon_stop.is_set():
            try:
                velocity_response = self.performer.motion.request(
                    NaoqiGetRobotVelocityRequest()
                )
                # Handle the response properly
                if hasattr(velocity_response, 'x') and hasattr(velocity_response, 'y') and hasattr(velocity_response, 'theta'):
                    vx, vy, vth = velocity_response.x, velocity_response.y, velocity_response.theta
                elif isinstance(velocity_response, (list, tuple)) and len(velocity_response) >= 3:
                    vx, vy, vth = velocity_response[0], velocity_response[1], velocity_response[2]
                else:
                    print(f"Unexpected velocity response format: {velocity_response}")
                    time.sleep(CHECK_DT)
                    continue
                # Check if the robot is moving
                moving = (abs(vx) + abs(vy) > VEL_THR) or (abs(vth) > 0.1)

                # If the robot is moving and collision protection is off, enable it
                if moving and self._prot_off:
                    print("Enabling collision protection")
                    self.performer.motion.request(
                        NaoqiCollisionProtectionRequest("Arms", True)
                    )
                    self._prot_off = False
                # If the robot is not moving and collision protection is on, disable it
                elif not moving and not self._prot_off:
                    print("Disabling collision protection")
                    self.performer.motion.request(
                        NaoqiCollisionProtectionRequest("Arms", False)
                    )
                    self._prot_off = True
            except Exception as e:
                print(f"Velocity monitor error: {e}")
            time.sleep(CHECK_DT)


    # ---------------------------------------------------------------------
    # Public entry point
    # ---------------------------------------------------------------------
    def run(self) -> None:
        print("Starting puppeteering session. Press <Enter> to stop.")
        # self._say("Start puppeteering.")
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
        # self._say("We are done puppeteering.")
        self._stop_streaming()
        try:
            # guarantee a hard stop
            self.performer.motion.request(
                NaoqiMoveTowardRequest(0.0, 0.0, 0.0)
            )
        except Exception:
            pass

        # Stop velocity monitoring
        self._stop_velocity_monitor()

        # Restore puppet stiffness before rest
        self.puppet.stiffness.request(
            Stiffness(0.7, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Re-enable autonomous life (optional but recommended for Pepper)
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("solitary"))
        self.performer.autonomous.request(NaoSetAutonomousLifeRequest("solitary"))

        # Put both robots into rest
        self.puppet.autonomous.request(NaoRestRequest())
        self.performer.autonomous.request(NaoRestRequest())

        print("Session ended; both Peppers are in rest pose.")


# ---------------------------------------------------------------------------
# Script entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    PepperPuppeteer(puppet_ip=PUPPET_IP, performer_ip=PERFORMER_IP).run()