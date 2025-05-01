import time

from sic_framework.devices import Pepper
from sic_framework.devices.common_naoqi.nao_motion_streamer import (
    NaoMotionStreamerConf,
    StartStreaming,
    StopStreaming,
)
from sic_framework.devices.common_naoqi.naoqi_autonomous import (
    NaoBackgroundMovingRequest,
    NaoBasicAwarenessRequest,
    NaoRestRequest,
    NaoSetAutonomousLifeRequest,
    NaoWakeUpRequest,
)
from sic_framework.devices.common_naoqi.naoqi_motion import (
    NaoPostureRequest,
    NaoqiAnimationRequest,
    NaoqiSmartStiffnessRequest,
)
from sic_framework.devices.common_naoqi.naoqi_stiffness import Stiffness
from sic_framework.devices.common_naoqi.naoqi_text_to_speech import (
    NaoqiTextToSpeechRequest,
)

JOINTS = ["Head", "RArm", "LArm"]
# FIXED_JOINTS = ["RLeg", "LLeg"]


class PuppeteerApplication:
    def __init__(self, master_ip="10.0.0.165", puppet_ip="10.0.0.196"):

        self.conf = NaoMotionStreamerConf(samples_per_second=30)
        # self.puppet_master = Pepper(
        #     master_ip,
        #     motion_stream_conf=self.conf,
        #     dev_test=True,
        #     test_repo="/home/karen/social-interaction-cloud",
        # )
        self.puppet_master = Pepper(
            master_ip, motion_stream_conf=self.conf, dev_test=True
        )

        # self.puppet = Pepper(
        #     puppet_ip,
        #     dev_test=True,
        #     test_repo="/home/karen/social-interaction-cloud",
        # )
        self.puppet = Pepper(puppet_ip, dev_test=True)

        self._setup_robots()

        self.is_paused = False

        # register callback for tactile sensor
        self.puppet_master.tactile_sensor.register_callback(self.on_touch)

    def _setup_robots(self):
        print(
            "----------------------------------Turn off the autonomous abilities on the master Pepper----------------------------------"
        )
        # Completely disable all autonomous capabilities (including obstacle avoidance) on the master Pepper to prevent interference
        # all the stiffness of its motors will be off, so it will slouch down
        self.puppet_master.autonomous.request(NaoSetAutonomousLifeRequest("disabled"))

        print(
            "----------------------------------Waking up robots----------------------------------"
        )
        # wake up robots (Important: for pepper, the stiffness can't be set when it is in rest mode, unlike nao)
        self.puppet_master.autonomous.request(NaoWakeUpRequest())
        self.puppet.autonomous.request(NaoWakeUpRequest())

        print(
            "----------------------------------Setting stiffness of robots----------------------------------"
        )

        # First, turn off the smart stiffness to avoid interference and make the movement of the master pepper easier
        self.puppet_master.motion.request(NaoqiSmartStiffnessRequest(False))
        self.puppet.motion.request(NaoqiSmartStiffnessRequest(False))

        # On Pepper, stiffness somehow can't be set at the individual joint level, so we just pass the chains instead of the joints
        self.puppet_master.stiffness.request(
            Stiffness(0.0, joints=JOINTS, enable_joint_list_generation=False)
        )

        # Set the stiffness of the puppet to 1.0 to control the joints fully
        self.puppet.stiffness.request(Stiffness(1, joints=JOINTS))

    def on_touch(self, message):
        if self.is_paused:
            self.puppet_master.motion_streaming.request(StartStreaming(JOINTS))
            self.is_paused = False
        else:
            self.puppet_master.motion_streaming.request(StopStreaming(JOINTS))
            self.is_paused = True

    def start_puppeteering(self, duration=30):
        print(
            "----------------------------------Starting puppeteering----------------------------------"
        )
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

        self.puppet_master.motion_streaming.request(StopStreaming())

        # Set both robots in rest pose again
        self.puppet_master.motion.request(NaoqiSmartStiffnessRequest(True))
        self.puppet_master.autonomous.request(NaoRestRequest())
        self.puppet.autonomous.request(NaoRestRequest())
        # self.puppet.stiffness.request(Stiffness(0, joints=JOINTS))
        # self.puppet_master.stiffness.request(Stiffness(0, joints=JOINTS))


if __name__ == "__main__":
    puppet_app = PuppeteerApplication(master_ip="10.0.0.165", puppet_ip="10.0.0.196")
    puppet_app.start_puppeteering()
