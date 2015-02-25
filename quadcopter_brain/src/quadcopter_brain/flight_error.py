class FlightError(Exception):
    def __init__(self, message="", copter=None):
        self.msg = message
        if copter:
            self.hover_in_place(copter)

    def __str__(self):
        return self.msg

    def hover_in_place(self, copter):
        print "Hovering in place"
        copter.hover_in_place()
