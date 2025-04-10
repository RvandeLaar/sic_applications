import time

from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoBackgroundMovingRequest,
    NaoBasicAwarenessRequest,
    NaoRestRequest,
)
from sic_framework.devices.common_naoqi.naoqi_motion_streamer import (
    NaoMotionStreamerConf,
    StartStreaming,
    StopStreaming,
)
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_text_to_speech import (
    NaoqiTextToSpeechRequest,
)

JOINTS = ["Head", "RArm", "LArm"]
FIXED_JOINTS = ["RLeg", "LLeg"]

class PuppeteerApplication:
    def __init__(self, master_ip="10.0.0.237", puppet_ip="10.0.0.238"):

        self.conf = NaoMotionStreamerConf(samples_per_second=30)
        self.puppet_master = Pepper(master_ip, motion_stream_conf=self.conf)
        self.puppet = Pepper(puppet_ip)
        
        self._setup_robots()
        
        self.is_paused = False
        
        # register callback for tactile sensor
        self.puppet_master.tactile_sensor.register_callback(self.on_touch)

    def _setup_robots(self):
        # Turn off basic awareness and background moving
        self.puppet_master.autonomous.request(NaoBasicAwarenessRequest(False))
        self.puppet_master.autonomous.request(NaoBackgroundMovingRequest(False))
        self.puppet_master.stiffness.request(Stiffness(stiffness=0.0, joints=JOINTS))

        self.puppet.autonomous.request(NaoBasicAwarenessRequest(False))
        self.puppet.autonomous.request(NaoBackgroundMovingRequest(False))
        self.puppet.stiffness.request(Stiffness(0.5, joints=JOINTS))

        # Set fixed joints to high stiffness
        self.puppet_master.stiffness.request(Stiffness(0.7, joints=FIXED_JOINTS))
        self.puppet.stiffness.request(Stiffness(0.7, joints=FIXED_JOINTS))

        # Start both robots in rest pose
        self.puppet.autonomous.request(NaoRestRequest())
        self.puppet_master.autonomous.request(NaoRestRequest())

    def on_touch(self, message):
        if self.is_paused:
            self.puppet_master.motion_streaming.request(StartStreaming())
            self.is_paused = False
        else:
            self.puppet_master.motion_streaming.request(StopStreaming())
            self.is_paused = True

    def start_puppeteering(self, duration=30):
        # Start the puppeteering and have Nao say that you can start
        self.puppet_master.motion_streaming.request(StartStreaming(JOINTS))

        # Input the puppet master's motion streamer into the puppet
        self.puppet.motion_streaming.connect(self.puppet_master.motion_streaming)

        self.puppet_master.tts.request(
            NaoqiTextToSpeechRequest("Start puppeteering", language="English", animated=True)
        )

        # Wait for puppeteering
        time.sleep(duration)

        # Cleanup
        self.puppet_master.tts.request(
            NaoqiTextToSpeechRequest(
                "We are done puppeteering", language="English", animated=True
            )
        )

        self.puppet_master.stiffness.request(Stiffness(0.7, joints=JOINTS))
        self.puppet_master.motion_streaming.request(StopStreaming())

        # Set both robots in rest pose again
        self.puppet.autonomous.request(NaoRestRequest())
        self.puppet_master.autonomous.request(NaoRestRequest())


if __name__ == "__main__":
    puppet_app = PuppeteerApplication(master_ip="10.0.0.237", puppet_ip="10.0.0.238")
    puppet_app.start_puppeteering()