from geometry_msgs.msg import Twist

class PIDHoldDepth:
    def __init__(self, params: dict):
        self.kp = float(params.get('kp', 0.8))
        self.ki = float(params.get('ki', 0.0))
        self.kd = float(params.get('kd', 0.05))
        self.setpoint = float(params.get('setpoint', 0.0))
        self.e_int = 0.0
        self.e_prev = 0.0
        self.last_t = None
        self.depth = 0.0  # TODO: update from sensor/topic

    def update(self, now_sec: float) -> Twist:
        if self.last_t is None:
            dt = 0.05
        else:
            dt = max(1e-3, now_sec - self.last_t)
        self.last_t = now_sec

        e = self.setpoint - self.depth
        self.e_int += e * dt
        de = (e - self.e_prev) / dt
        self.e_prev = e

        u = self.kp * e + self.ki * self.e_int + self.kd * de

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = max(-1.0, min(1.0, -u))  # positive up
        cmd.angular.z = 0.0
        return cmd

    def on_state(self, state_msg):
        pass
