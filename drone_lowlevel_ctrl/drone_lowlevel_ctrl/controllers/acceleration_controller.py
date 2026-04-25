class AccelerationController:
    def __init__(self, base_thrust=1.0, g= 9.81):
        self.base_thrust= base_thrust
        self.g= g

    def compute(self, a_x, a_y, a_z):
        roll_des= a_y/ self.g
        pitch_des= -a_x/self.g
        thrust= self.base_thrust +a_z

        #I am clamping to avoid unstable values

        roll_des= max(-0.3, min(0.3, roll_des))
        pitch_des= max(-0.3, min(0.3, pitch_des))
        thrust= max(0.0, min(1.0, thrust))

        return roll_des, pitch_des, thrust



