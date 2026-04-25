class AttitudeController:
    def __init__(self, kp=1.5):
        self.kp = kp

    def compute(self, roll_des, pitch_des, roll_curr, pitch_curr):
    
        roll_error = roll_des - roll_curr
        pitch_error = pitch_des - pitch_curr

        roll_cmd = self.kp * roll_error
        pitch_cmd = self.kp * pitch_error

        #I am clamping the output for the safety of the drone 

        roll_cmd= max(-0.5, min(0.5, roll_cmd))
        pitch_cmd= max(-0.5, min(0.5, pitch_cmd))

        yaw_cmd = 0.0 

        return roll_cmd, pitch_cmd, yaw_cmd