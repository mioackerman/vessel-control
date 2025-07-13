class PID:
    def __init__(self, Kp, Ki, Kd, output_limits=(0.0, 1.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
        self.output_limits = output_limits

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return max(self.output_limits[0], min(self.output_limits[1], output))
