#!/usr/bin/env python3
"""
Pepper-to-Pepper puppeteering demo (Baseline)
---------------------------------------------
  • PUPPET      : Pepper moved manually by the operator
  • PERFORMER   : Pepper that mirrors the puppet's joint positions

Controls
--------
  • Press <Enter> (or Ctrl-C) in the terminal to finish the session

This is the baseline implementation for comparison with the enhanced version.
Both robots are always returned to a safe rest pose when the program exits.
"""

# External imports
import csv
import datetime
import os
import time

# SIC imports
from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoSetAutonomousLifeRequest,
    NaoWakeUpRequest,
    NaoRestRequest,
)
from sic_framework.devices.common_naoqi.naoqi_text_to_speech import NaoqiTextToSpeechRequest
from sic_framework.devices.common_naoqi.pepper_motion_streamer import (
    PepperMotionStreamerConf,
    StartStreaming,
    StopStreaming,
)
from sic_framework.devices.common_naoqi.naoqi_motion import (
    NaoqiSmartStiffnessRequest,
    NaoqiBreathingRequest,
)

# ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
# Configuration
# ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
# Robot IPs
PUPPET_IP = "10.0.0.168"
PERFORMER_IP = "10.0.0.196"

# Motion streaming configuration
ACTIVE_JOINTS = ["Head", "RArm", "LArm"]     # motion chains to stream
STREAM_HZ = 30                               # samples per second

# Timing configuration
TIMING_FILE = "baseline_puppeteering_times.csv"

# ─────────────────────────────────────────────────────────────────────────────
# Puppeteering Class
# ─────────────────────────────────────────────────────────────────────────────
class PepperPuppeteer:
    """Encapsulates the two robots and all puppeteering logic."""

    def __init__(self, puppet_ip: str, performer_ip: str):
        # Motion-streamer configurations
        puppet_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=0.0)
        performer_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=1.0)

        self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf, dev_test=True)
        self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf, dev_test=True)

        # Timing variables
        self.session_start_time = None
        self.session_duration = None

        # Initialize robots
        self._initialise_robots()

    # ---------------------------------------------------------------------
    # Initialisation
    # ---------------------------------------------------------------------
    def _initialise_robots(self):
        """Initialize both robots for puppeteering.
        
        This method sets up both robots for safe puppeteering operation by:
        1. Disabling autonomous behaviors that could interfere with manual control
        2. Waking up the robots (required for stiffness control)
        3. Disabling smart stiffness and breathing animations
        4. Setting appropriate initial stiffness values
        5. Connecting the motion streaming between robots
        """
        # Disable autonomous behaviour that interferes with manual control
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))
        self.performer.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))

        # Wake up (Pepper cannot change stiffness while in rest)
        self.puppet.autonomous.request(NaoWakeUpRequest())
        self.performer.autonomous.request(NaoWakeUpRequest())

        # Disable smart stiffness and breathing for proper teleoperation
        self.puppet.motion.request(NaoqiSmartStiffnessRequest(False))
        self.performer.motion.request(NaoqiSmartStiffnessRequest(False))
        self.puppet.motion.request(NaoqiBreathingRequest("Arms", False))
        self.performer.motion.request(NaoqiBreathingRequest("Arms", False))

        # Puppet: zero stiffness on streamed chains so the operator can move joints
        self.puppet.stiffness.request(
            Stiffness(0, joints=ACTIVE_JOINTS, enable_joint_list_generation=False)
        )
        # Connect performer's motion-stream input to the master's output
        self.performer.motion_streaming.connect(self.puppet.motion_streaming)

    # ---------------------------------------------------------------------
    # Timing functions
    # ---------------------------------------------------------------------
    def _start_timing(self):
        """Start timing the puppeteering session."""
        self.session_start_time = time.time()
        print(f"Session started at: {datetime.datetime.now().strftime('%d-%m-%Y, %H:%M')}")

    def _stop_timing(self):
        """Stop timing and calculate session duration."""
        if self.session_start_time is not None:
            self.session_duration = time.time() - self.session_start_time
            duration_str = self._format_duration(self.session_duration)
            print(f"Session duration: {duration_str}")
            return duration_str
        return "0:00"

    def _format_duration(self, duration_seconds):
        """Format duration in seconds to MM:SS format."""
        minutes = int(duration_seconds // 60)
        seconds = int(duration_seconds % 60)
        return f"{minutes}:{seconds:02d}"

    def _save_timing_data(self):
        """Save timing data to CSV file."""
        if self.session_start_time is None:
            return

        # Create file with header if it doesn't exist
        file_exists = os.path.exists(TIMING_FILE)
        
        with open(TIMING_FILE, 'a', newline='') as csvfile:
            fieldnames = ['datetime', 'duration']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            if not file_exists:
                writer.writeheader()
            
            # Write the timing data
            writer.writerow({
                'datetime': datetime.datetime.now().strftime('%d-%m-%Y, %H:%M'),
                'duration': self._format_duration(self.session_duration)
            })
        
        print(f"Timing data saved to {TIMING_FILE}")

    # ---------------------------------------------------------------------
    # Convenience wrappers
    # ---------------------------------------------------------------------
    def _say(self, text: str) -> None:
        """
        Make the puppet speak the given text.
        """
        self.puppet.tts.request(
            NaoqiTextToSpeechRequest(text, language="English")
        )

    def _start_streaming(self) -> None:        
        """
        Start motion streaming between robots.
        """
        self.puppet.motion_streaming.request(StartStreaming(ACTIVE_JOINTS))

    def _stop_streaming(self) -> None:
        """
        Stop motion streaming between robots.
        """
        self.puppet.motion_streaming.request(StopStreaming())

    # ---------------------------------------------------------------------
    # Public entry point
    # ---------------------------------------------------------------------
    def run(self) -> None:
        """
        Run the puppeteering session.
        
        This method starts the puppeteering session and waits for user input
        to stop. It provides a clean shutdown when interrupted.
        """
        print("Starting puppeteering session. Press <Enter> to stop.")
        self._start_timing()  # Start timing
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
        """
        Clean shutdown of the puppeteering session.
        
        This method ensures both robots are returned to a safe state:
        1. Stops motion streaming
        2. Restores appropriate stiffness values
        3. Re-enables autonomous life
        4. Puts robots into rest pose
        5. Saves timing data
        """
        self._say("We are done puppeteering.")
        self._stop_streaming()

        # Stop timing and save data
        duration = self._stop_timing()
        self._save_timing_data()

        # Restore puppet stiffness before rest
        self.puppet.stiffness.request(
            Stiffness(0.7, joints=ACTIVE_JOINTS, enable_joint_list_generation=False)
        )

        # Re-enable autonomous life (optional but recommended for Pepper)
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("solitary"))

        # Put both robots into rest
        self.puppet.autonomous.request(NaoRestRequest())
        self.performer.autonomous.request(NaoRestRequest())

        print("Session ended; both Peppers are in rest pose.")


# ─────────────────────────────────────────────────────────────────────────────
# Script entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    PepperPuppeteer(puppet_ip=PUPPET_IP, performer_ip=PERFORMER_IP).run()