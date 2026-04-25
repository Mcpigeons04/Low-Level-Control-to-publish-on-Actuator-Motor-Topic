class VelocityController:
    def __init__(self, kp=0.5):
        self.kp=kp

    def compute(self, v_des, v_current):
        error= v_des-v_current
        a_des= self.kp*(error)
        return a_des

