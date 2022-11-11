class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.__error = 0.0
        self.__prev_error = 0.0
        self.__integral = 0.0
        self.__derivative = 0.0

    def state_as_str(self) -> str:
        return f"Error: {self.__error}, Integral: {self.__integral}, Derivative: {self.__derivative}"

    def update(self, desired: float, actual: float, dt: float) -> float:
        self.__error = desired - actual
        self.__integral += self.__error * dt
        self.__derivative = (self.__error - self.__prev_error) / dt
        self.__prev_error = self.__error
        return (
            self.kp * self.__error + 
            self.ki * self.__integral + 
            self.kd * self.__derivative
        )