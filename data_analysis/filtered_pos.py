class FilteredPos(object):
    def __init__(self, data):
        self.time = data[0]
        self.seq = data[1]
        self.stamp = data[2]
        self.frame_id = data[3]
        self.latitude = data[4]
        self.longitude = data[5]
        self.altitude = data[6]
        self.relative_altitude = data[7]
        self.ground_x_speed = data[8]
        self.ground_y_speed = data[9]
        self.ground_z_speed = data[10]
        self.heading = data[11]
