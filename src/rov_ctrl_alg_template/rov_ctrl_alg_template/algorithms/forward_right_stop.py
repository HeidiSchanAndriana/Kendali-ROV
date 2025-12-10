from geometry_msgs.msg import Twist

class ForwardRightStop:
    """
    Maju t_forward detik (x=+surge), lalu geser kanan t_right detik (y=+sway), lalu netral.
    Nilai input [-1..1], akan di-scale ControlMux ke MANUAL_CONTROL.
    Default: surge=0.6, sway=0.5, t_forward=3.0, t_right=3.0
    """
    def __init__(self, params: dict):
        self.surge = float(params.get('surge', 0.6))
        self.sway = float(params.get('sway', 0.5))
        self.t_forward = float(params.get('t_forward', 3.0))
        self.t_right = float(params.get('t_right', 3.0))

    def update(self, now_sec: float) -> Twist:
        cmd = Twist()
        if now_sec < self.t_forward:
            cmd.linear.x = self.surge
        elif now_sec < self.t_forward + self.t_right:
            cmd.linear.y = self.sway
        # lainnya netral (0)
        return cmd
