import time

from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoBackgroundMovingRequest,
    NaoBasicAwarenessRequest,
    NaoRestRequest,
)
from sic_framework.devices.common_naoqi.nao_motion_streamer import (
    NaoMotionStreamerConf,
    StartStreaming,
    StopStreaming,
)
from sic_framework.devices.common_naoqi.naoqi_motion import (
    NaoPostureRequest,
    NaoqiAnimationRequest,
)
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_text_to_speech import (
    NaoqiTextToSpeechRequest,
)
from sic_framework.devices.common_naoqi.naoqi_autonomous import NaoWakeUpRequest, NaoRestRequest


JOINTS = ["Head", "RArm", "LArm"]
# FIXED_JOINTS = ["RLeg", "LLeg"]

class PuppeteerApplication:
    def __init__(self, master_ip="10.0.0.148", puppet_ip="10.0.0.196"):

        self.conf = NaoMotionStreamerConf(samples_per_second=30)
        self.puppet_master = Pepper(master_ip, motion_stream_conf=self.conf, dev_test=True, test_repo="/Users/apple/Desktop/SAIL/SIC_Development/social-interaction-cloud")
        # self.puppet_master = Pepper(master_ip, motion_stream_conf=self.conf, dev_test=True)

        self.puppet = Pepper(puppet_ip, dev_test=True, test_repo="/Users/apple/Desktop/SAIL/SIC_Development/social-interaction-cloud")
        # self.puppet = Pepper(puppet_ip, dev_test=True)
        
        self._setup_robots()
        
        self.is_paused = False
        
        # register callback for tactile sensor
        self.puppet_master.tactile_sensor.register_callback(self.on_touch)

    def _setup_robots(self):

        print("----------------------------------Waking up robots----------------------------------")
        # wake up robots
        # self.puppet_master.autonomous.request(NaoWakeUpRequest())
        # self.puppet.autonomous.request(NaoWakeUpRequest())
        self.puppet_master.motion.request(NaoPostureRequest("Stand", 0.5))
        self.puppet.motion.request(NaoPostureRequest("Stand", 0.5))


        print("----------------------------------Setting stiffness of robots----------------------------------")
        # Turn off basic awareness and background moving
        # self.puppet_master.autonomous.request(NaoBasicAwarenessRequest(False))
        # self.puppet_master.autonomous.request(NaoBackgroundMovingRequest(False))
        self.puppet_master.stiffness.request(Stiffness(0.0, joints=JOINTS))
        
        # self.puppet.autonomous.request(NaoBasicAwarenessRequest(False))
        # self.puppet.autonomous.request(NaoBackgroundMovingRequest(False))
        self.puppet.stiffness.request(Stiffness(0.5, joints=JOINTS))

        # Start both robots in rest pose
        # self.puppet.autonomous.request(NaoRestRequest())
        # self.puppet_master.autonomous.request(NaoRestRequest())

    def on_touch(self, message):
        if self.is_paused:
            self.puppet_master.motion_streaming.request(StartStreaming(JOINTS))
            self.is_paused = False
        else:
            self.puppet_master.motion_streaming.request(StopStreaming(JOINTS))
            self.is_paused = True

    def start_puppeteering(self, duration=30):
        print("----------------------------------Starting puppeteering----------------------------------")
        # Input the puppet master's motion streamer into the puppet
        self.puppet.motion_streaming.connect(self.puppet_master.motion_streaming)
       
        # Start the puppeteering and have Nao say that you can start
        self.puppet_master.motion_streaming.request(StartStreaming(JOINTS))

        self.puppet_master.tts.request(
            NaoqiTextToSpeechRequest("Start puppeteering", language="English")
        )

        # Wait for puppeteering
        time.sleep(duration)

        # Cleanup
        self.puppet_master.tts.request(
            NaoqiTextToSpeechRequest("We are done puppeteering", language="English")
        )

        # self.puppet_master.stiffness.request(Stiffness(0.7, joints=JOINTS))
        self.puppet_master.motion_streaming.request(StopStreaming())

        # Set both robots in rest pose again
        self.puppet_master.autonomous.request(NaoRestRequest())
        self.puppet.autonomous.request(NaoRestRequest())


if __name__ == "__main__":
    puppet_app = PuppeteerApplication(master_ip="10.0.0.148", puppet_ip="10.0.0.196")
    puppet_app.start_puppeteering()