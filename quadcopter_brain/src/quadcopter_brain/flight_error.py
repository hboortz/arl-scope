class FlightError(Exception):
    def __init__(self, message="", copter=None):
        self.msg = message
        if copter:
            self.hover_in_place(copter)
        # shutdown rospy node
        # gc literally everything
        # kill forked processes

    def __str__(self):
        return self.msg

    def hover_in_place(self, copter):
    	print "Trying to hover in place"
        copter.hover_in_place()
