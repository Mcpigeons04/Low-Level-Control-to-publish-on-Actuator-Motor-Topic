class RateController:
    def __init__(self, kd=0.1):
        self.kd = kd

    def compute(self, roll_cmd, pitch_cmd, yaw_cmd, ang_vel):
        roll_rate = ang_vel[0]
        pitch_rate = ang_vel[1]
        yaw_rate = ang_vel[2]

        # damping after attitude controller
        roll_cmd  = roll_cmd  - self.kd * roll_rate
        pitch_cmd = pitch_cmd - self.kd * pitch_rate
        yaw_cmd   = yaw_cmd   - self.kd * yaw_rate

        return roll_cmd, pitch_cmd, yaw_cmd