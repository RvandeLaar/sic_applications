#!/usr/bin/env python3
"""
Pepper-to-Pepper puppeteering demo
---------------------------------
  • PUPPET      : Pepper moved manually by the operator.
  • PERFORMER   : Pepper that mirrors the puppet's joint positions.

Controls
--------
  • Back bumper: Pause/resume puppeteering
  • Right bumper: Lock/unlock right arm
  • Left bumper: Lock/unlock left arm
  • Head sensor: Lock/unlock head
  • Press <Enter> (or Ctrl-C) in the terminal to finish the session.

Both robots are always returned to a safe rest pose when the program exits.
"""

# External imports
import os
import threading
import time
from collections import defaultdict

# SIC imports
from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.naoqi_leds import NaoFadeRGBRequest
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoRestRequest,
    NaoSetAutonomousLifeRequest,
    NaoWakeUpRequest,
)
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
    NaoqiGetAnglesRequest,
)

# ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
# Configuration
# ────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
# Set Redis password environment variable (when in developer mode)
os.environ['REDIS_PASSWORD'] = 'changemeplease'  # Do NOT actually change it
# Set Redis IP for robots to connect to your laptop (when in developer mode)
os.environ['DB_IP'] = '10.0.0.127'  # Your laptop's IP address

# Robot IPs
PUPPET_IP = "10.0.0.168"
PERFORMER_IP = "10.0.0.196"

# Motion streaming configuration
JOINTS = ["Head", "RArm", "LArm"]            # motion chains to stream
STREAM_HZ = 30                               # samples per second
VEL_THR = 0.02                               # m/s, below this we treat the robot as "still"
CHECK_DT = 0.2                               # seconds, how often to check velocity

# Blockage detection configuration
CRITICAL_JOINTS = {
    "LArm": ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"],
    "RArm": ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"],
}

BLOCKAGE_CONFIG = {
    "angle_threshold": 0.25,                # radians - mismatch threshold
    "sustained_time": 0.5,                  # seconds - time to confirm blockage
    "blockage_ratio": 0.70,                 # fraction of recent samples that must show blockage
}

# ─────────────────────────────────────────────────────────────────────────────
# Puppeteering Class
# ─────────────────────────────────────────────────────────────────────────────
class PepperPuppeteer:
    """Encapsulates the two robots and all puppeteering logic."""

    def __init__(self, puppet_ip: str, performer_ip: str):
        # Motion-streamer configurations
        puppet_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=0.0)
        performer_conf = PepperMotionStreamerConf(samples_per_second=STREAM_HZ, stiffness=1.0)

        # Use dev_test mode to test new implementations
        # self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf, dev_test=True, test_repo="/Users/rvandelaar/Desktop/School/Master AI Thesis/MSc_AI_Thesis/social-interaction-cloud")
        # self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf, dev_test=True, test_repo="/Users/rvandelaar/Desktop/School/Master AI Thesis/MSc_AI_Thesis/social-interaction-cloud")

        # Once installed, you can switch to:
        self.puppet = Pepper(puppet_ip, pepper_motion_conf=puppet_conf, dev_test=True)
        self.performer = Pepper(performer_ip, pepper_motion_conf=performer_conf, dev_test=True)

        # Threading locks
        self._blockage_lock = threading.Lock()
        self._angle_lock = threading.Lock()

        # State variables
        self._paused = False                        # True if the puppeteering is paused
        self._prot_off = False                      # True if collision protection is off
        self._vel_mon_stop = threading.Event()      # Event to stop velocity monitoring
        self._vel_mon_thr = None                    # Thread to monitor velocity
        
        # Joint locking state
        self._locked_joints = set()                 # Set of locked joint chains
        self._locked_angles = {}                    # Angles to maintain for locked joints

        # Blockage detection state
        self._puppet_angles = {}                    # Latest puppet joint angles
        self._performer_angles = {}                 # Latest performer joint angles
        self._hist = defaultdict(list)              # per-chain time-series
        self._active_blockages = set()              # chains currently blocked

        # Initialize robots
        self._initialise_robots()

        # Start background threads
        threading.Thread(target=self._blockage_watchdog, daemon=True).start()

        # Register sensor callbacks
        self._register_sensor_callbacks()

        # Start velocity monitoring
        self._start_velocity_monitor()

    # ---------------------------------------------------------------------
    # Initialisation
    # ---------------------------------------------------------------------
    def _initialise_robots(self):
        """Initialize both robots for puppeteering."""
        # Disable autonomous behaviour that interferes with manual control
        self.puppet.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))
        self.performer.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))

        # Wake up (Pepper cannot change stiffness while in rest)
        self.puppet.autonomous.request(NaoWakeUpRequest())
        self.performer.autonomous.request(NaoWakeUpRequest())

        # Puppet: zero stiffness on streamed chains so the operator can move joints
        self.puppet.stiffness.request(
            Stiffness(0, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Connect performer's motion-stream input to the master's output
        self.performer.motion_streaming.connect(self.puppet.motion_streaming)

        self._register_stream_callback()

    def _register_sensor_callbacks(self):
        """Register callbacks for all sensor inputs."""
        sensors = [
            (self.puppet.back_bumper, self._on_back_bumper, "Back bumper"),
            (self.puppet.right_bumper, self._on_right_bumper, "Right bumper"),
            (self.puppet.left_bumper, self._on_left_bumper, "Left bumper"),
            (self.puppet.tactile_sensor, self._on_head_touch, "Head sensor"),
        ]
        
        for sensor, callback, name in sensors:
            try:
                sensor.register_callback(callback)
                print(f"✓ {name} callback registered successfully")
            except Exception as e:
                print(f"✗ Error registering {name} callback: {e}")

    # ---------------------------------------------------------------------
    # Convenience wrappers
    # ---------------------------------------------------------------------
    def _say(self, text: str) -> None:
        """Make the puppet speak the given text."""
        self.puppet.tts.request(
            NaoqiTextToSpeechRequest(text, language="English")
        )

    def _start_streaming(self) -> None:        
        """Start motion streaming between robots."""
        self.puppet.motion_streaming.request(StartStreaming(JOINTS))

    def _stop_streaming(self) -> None:
        """Stop motion streaming between robots."""
        self.puppet.motion_streaming.request(StopStreaming(JOINTS))
    
    def _pause_puppeteering(self) -> None:
        """Pause all puppeteering (joints and base)."""
        self._stop_streaming()
        # Set velocity to 0 to stop base movement
        self.performer.motion.request(NaoqiMoveTowardRequest(0.0, 0.0, 0.0))
        # Clear blockage state since we're pausing
        self._clear_blockage_state()
    
    def _resume_puppeteering(self) -> None:
        """Resume all puppeteering (joints and base)."""
        self._start_streaming()
    
    def _lock_joint(self, joint_chain: str) -> None:
        """Lock a joint chain in its current position."""
        if joint_chain not in self._locked_joints:
            self._locked_joints.add(joint_chain)
            
            # Store current angles for this joint chain
            with self._angle_lock:
                if joint_chain in CRITICAL_JOINTS:
                    # For arms, store the current angles
                    for joint in CRITICAL_JOINTS[joint_chain]:
                        if joint in self._performer_angles:
                            self._locked_angles[joint] = self._performer_angles[joint]
                elif joint_chain == "Head":
                    # For head, store current head angles
                    head_joints = ["HeadYaw", "HeadPitch"]
                    for joint in head_joints:
                        if joint in self._performer_angles:
                            self._locked_angles[joint] = self._performer_angles[joint]
            
            print(f"Locked {joint_chain} at angles: {self._locked_angles}")
    
    def _unlock_joint(self, joint_chain: str) -> None:
        """Unlock a joint chain to allow normal puppeteering."""
        if joint_chain in self._locked_joints:
            self._locked_joints.remove(joint_chain)
            
            # Remove stored angles for this joint chain
            if joint_chain in CRITICAL_JOINTS:
                for joint in CRITICAL_JOINTS[joint_chain]:
                    self._locked_angles.pop(joint, None)
            elif joint_chain == "Head":
                head_joints = ["HeadYaw", "HeadPitch"]
                for joint in head_joints:
                    self._locked_angles.pop(joint, None)
            
            print(f"Unlocked {joint_chain}")
    
    # ---------------------------------------------------------------------
    # Chest LED update
    # ---------------------------------------------------------------------
    def _update_chest_led(self):
        """Update chest LED color based on blockage state."""
        if any(chain in self._active_blockages for chain in CRITICAL_JOINTS.keys()):
            color = (1.0, 0.0, 0.0)  # red light to indicate blockage
        else:
            color = (0.0, 0.0, 0.0)  # lights off when blockage no longer detected
        try:
            self.puppet.leds.request(NaoFadeRGBRequest("ChestLeds", *color))
        except Exception as e:
            print("LED error:", e)
    
    # -------------------------------------------------------------------------
    # Motion-stream callback: keeps _puppet_angles current
    # -------------------------------------------------------------------------
    def _register_stream_callback(self):
        """Register callback for motion streaming."""
        self.puppet.motion_streaming.register_callback(self._on_stream_packet)

    def _on_stream_packet(self, msg):
        """Handle incoming motion stream packets."""
        try:
            with self._angle_lock:
                # Update puppet angles
                self._puppet_angles.update(dict(zip(msg.joints, msg.angles)))
                
                # Override angles for locked joints
                for joint_chain in self._locked_joints:
                    if joint_chain in CRITICAL_JOINTS:
                        # For arms, override with locked angles
                        for joint in CRITICAL_JOINTS[joint_chain]:
                            if joint in self._locked_angles:
                                self._puppet_angles[joint] = self._locked_angles[joint]
                    elif joint_chain == "Head":
                        # For head, override with locked angles
                        head_joints = ["HeadYaw", "HeadPitch"]
                        for joint in head_joints:
                            if joint in self._locked_angles:
                                self._puppet_angles[joint] = self._locked_angles[joint]
                                
        except Exception as e:
            print("Stream-callback error:", e)

    # ---------------------------------------------------------------------
    # Sensor callbacks
    # ---------------------------------------------------------------------
    def _on_back_bumper(self, msg) -> None:
        """Handle back bumper press - toggle pause/resume."""
        print(f"Back bumper pressed: value={msg.value}, paused={self._paused}")
        if msg.value != 1:      # ignore release
            return

        if self._paused:        # → RESUME
            try:
                self._resume_puppeteering()
            except Exception as e:
                print("Error resuming puppeteering:", e)
            else:
                self._say("Resuming puppeteering.")
                self._paused = False
        else:                   # → PAUSE
            try:
                self._pause_puppeteering()
            except Exception as e:
                print("Error pausing puppeteering:", e)
            else:
                self._say("Pausing puppeteering.")
                self._paused = True
    
    def _on_right_bumper(self, msg) -> None:
        """Handle right bumper press - toggle right arm lock."""
        print(f"Right bumper pressed: value={msg.value}")
        if msg.value != 1:      # ignore release
            return
        
        if "RArm" in self._locked_joints:  # → UNLOCK
            self._unlock_joint("RArm")
            self._say("Right arm unlocked.")
        else:                              # → LOCK
            self._lock_joint("RArm")
            self._say("Right arm locked.")
    
    def _on_left_bumper(self, msg) -> None:
        """Handle left bumper press - toggle left arm lock."""
        print(f"Left bumper pressed: value={msg.value}")
        if msg.value != 1:      # ignore release
            return
        
        if "LArm" in self._locked_joints:  # → UNLOCK
            self._unlock_joint("LArm")
            self._say("Left arm unlocked.")
        else:                              # → LOCK
            self._lock_joint("LArm")
            self._say("Left arm locked.")
    
    def _on_head_touch(self, msg) -> None:
        """Handle head sensor touch - toggle head lock."""
        print(f"Head sensor touched: value={msg.value}")
        if msg.value != 1:      # ignore release
            return
        
        if "Head" in self._locked_joints:  # → UNLOCK
            self._unlock_joint("Head")
            self._say("Head unlocked.")
        else:                              # → LOCK
            self._lock_joint("Head")
            self._say("Head locked.")
    
    # ---------------------------------------------------------------------
    # Blockage detection
    # ---------------------------------------------------------------------
    def _analyse_blockages(self, now):
        """Analyze joint blockages and update state."""
        with self._angle_lock:
            puppet_snapshot = self._puppet_angles.copy()
            performer_snapshot = self._performer_angles.copy()
        
        H = self._hist
        prev_blockages = set(self._active_blockages)  # Save previous state

        for chain, joints in CRITICAL_JOINTS.items():
            # max absolute error among that chain's joints
            err = max(
                abs(puppet_snapshot.get(j, 0) - performer_snapshot.get(j, 0))
                for j in joints
            )

            blocked = err > BLOCKAGE_CONFIG["angle_threshold"]
            H[chain].append((now, blocked))

            # forget stale samples
            cutoff = now - BLOCKAGE_CONFIG["sustained_time"]
            H[chain] = [(t, b) for (t, b) in H[chain] if t >= cutoff]

            # decide persistent blockage
            if H[chain]:
                ratio = sum(b for _, b in H[chain]) / float(len(H[chain]))
                pers = ratio >= BLOCKAGE_CONFIG["blockage_ratio"]
                if pers and chain not in self._active_blockages:
                    self._active_blockages.add(chain)
                    self._say(f"{chain} mismatch")
                    print(f"*** {chain} BLOCKAGE DETECTED ***")
                elif not pers and chain in self._active_blockages:
                    self._active_blockages.remove(chain)
                    print(f"*** {chain} BLOCKAGE CLEARED ***")

        # If the set of blockages changed, update the chest LED
        if self._active_blockages != prev_blockages:
            self._update_chest_led()

    # ---------------------------------------------------------------------
    # Velocity monitoring
    # ---------------------------------------------------------------------
    def _start_velocity_monitor(self):
        """Start velocity monitoring thread."""
        if self._vel_mon_thr and self._vel_mon_thr.is_alive():
            return
        self._vel_mon_stop.clear()
        self._vel_mon_thr = threading.Thread(
            target=self._velocity_loop, daemon=True
        )
        self._vel_mon_thr.start()

    def _stop_velocity_monitor(self):
        """Stop velocity monitoring thread."""
        self._vel_mon_stop.set()
        if self._vel_mon_thr:
            self._vel_mon_thr.join()

    def _velocity_loop(self):
        """Monitor robot velocity and manage collision protection."""
        while not self._vel_mon_stop.is_set():
            try:
                velocity_response = self.performer.motion.request(
                    NaoqiGetRobotVelocityRequest()
                )
                # Handle the response properly
                if hasattr(velocity_response, 'x'):
                    vx, vy, vth = velocity_response.x, velocity_response.y, velocity_response.theta
                else:
                    vx, vy, vth = velocity_response[0], velocity_response[1], velocity_response[2]
                
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

    # -------------------------------------------------------------------------
    # Blockage watchdog – runs at ~5 Hz, independent of streaming rate
    # -------------------------------------------------------------------------
    def _blockage_watchdog(self):
        """Monitor for joint blockages at 5 Hz."""
        SAMPLE_PERIOD = 0.20                      # 5 Hz
        names = [j for chain in ("LArm", "RArm") for j in CRITICAL_JOINTS[chain]]
        while True:
            t0 = time.time()

            # -------- gather angles from performer -------------
            try:
                with self._blockage_lock:
                    response = self.performer.motion.request(NaoqiGetAnglesRequest(names, True))
                    sensed = response.angles
                with self._angle_lock:
                    self._performer_angles = dict(zip(names, sensed))
            except Exception as e:
                print(f"Error getting performer angles: {e}")
                # robot may be booting – just skip this cycle
                time.sleep(SAMPLE_PERIOD)
                continue

            # -------- compare with puppet commanded angles ----
            self._analyse_blockages(time.time())

            # pacing
            dt = time.time() - t0
            if dt < SAMPLE_PERIOD:
                time.sleep(SAMPLE_PERIOD - dt)

    # ---------------------------------------------------------------------
    # Public entry point
    # ---------------------------------------------------------------------
    def run(self) -> None:
        """Main entry point for the puppeteering session."""
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
    def _clear_blockage_state(self):
        """Clear all blockage and locking state."""
        self._hist.clear()
        self._active_blockages.clear()
        self._update_chest_led()
        
        # Clear locked joints when pausing
        self._locked_joints.clear()
        self._locked_angles.clear()

    def _shutdown(self) -> None:
        """Clean shutdown of the puppeteering session."""
        self._say("We are done puppeteering.")
        self._clear_blockage_state()
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