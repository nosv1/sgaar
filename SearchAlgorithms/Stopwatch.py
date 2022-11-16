import time

class Stopwatch:
    def __init__(self) -> None:
        self.start_time = None
        self.end_time = None
    
    @property
    def elapsed_time(self) -> float:
        """
        Returns the ellapsed time in seconds
        """
        return self.end - self.start_time

    def start(self) -> None:
        """
        Starts the stopwatch
        """
        self.start_time = time.perf_counter()

    def stop(self) -> None:
        """
        Stops the stopwatch
        """
        self.end = time.perf_counter()