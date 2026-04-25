class Mixer:
    def mix(self, thrust, roll, pitch, yaw):
        m1 = thrust + roll + pitch + yaw
        m2 = thrust - roll + pitch - yaw
        m3 = thrust - roll - pitch + yaw
        m4 = thrust + roll - pitch - yaw

        motors = [m1, m2, m3, m4]

        max_val = max(motors)
        if max_val > 1.0:
            motors = [m / max_val for m in motors]

        # clamp safety for drone
        motors = [max(0.0, min(1.0, m)) for m in motors]

        return motors + [0.0]*8