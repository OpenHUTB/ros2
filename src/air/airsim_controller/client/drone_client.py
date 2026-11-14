
import airsim

from client.airsim_client import AirsimClient


class DroneClient(AirsimClient):
    def __init__(self, interval, root_path='./'):
        super(DroneClient, self).__init__(interval, root_path)
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def destroy(self):
        self.client.enableApiControl(False)

    def start(self):
        self.client.takeoffAsync().join()

    def get_state(self):
        return self.client.getMultirotorState()

    def get_collision_info(self):
        return self.client.simGetCollisionInfo()

    def get_images(self, camera_number='0'):
        responses = self.client.simGetImages([
            airsim.ImageRequest(camera_number, airsim.ImageType.Scene, False, False),  # png
            airsim.ImageRequest(camera_number, airsim.ImageType.DepthVis, True, False),  # depth visualization image
            airsim.ImageRequest(camera_number, airsim.ImageType.DepthPerspective, True, False),  # depth in perspective projection
        ])
        return responses

    def move(self, move_type, *args):
        if move_type == 'position':
            self._go_to_loc(*args)
        elif move_type == 'velocity':
            self._move_by_velocity(*args)
        else:
            raise NotImplementedError()

    def _go_to_loc(self, mx, my, mz, velocity):
        self.client.moveToPositionAsync(mx, my, mz, velocity).join()

    def _move_by_velocity(self, vx, vy, vz):
        self.client.moveByVelocityAsync(vx, vy, vz, self.interval).join()
