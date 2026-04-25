class PositionController:
    def __init__(self, kp=0.8):
        self.kp=kp

    def compute(self,target_z, current_z):
        error= current_z - target_z
        v_des= self.kp*(error)
        return v_des