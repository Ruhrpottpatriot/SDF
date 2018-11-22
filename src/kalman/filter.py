from numpy import ndarray, array, int32

class KalmanFilter(object):
    def __init__(self, sensor: Sensor) -> None:
        self.sensor = sensor

    def predict(self, timestep: int32) -> ndarray:
        pass # Predict object position for each object

    def filter(self, timestep: int32) -> ndarray:
        pass # Filter object position for each measurement

    def retrodict(self, timestep: int32) ->ndarray:
        pass # Apply measurements to all past positions