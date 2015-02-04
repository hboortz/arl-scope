class FilteredPos(object):
    def __init__(self, data):
        self.time = float(data[0]) / 1e9  # seconds
        self.seq = int(data[1])
        self.stamp = int(data[2])
        self.latitude = float(data[4]) / 1e7  # GPS decimal degrees
        self.longitude = float(data[5]) / 1e7  # GPS decimal degrees
        self.altitude = int(data[6])
        self.relative_altitude = int(data[7])
        self.ground_x_speed = int(data[8])
        self.ground_y_speed = int(data[9])
        self.ground_z_speed = int(data[10])
        self.heading = int(data[11])
