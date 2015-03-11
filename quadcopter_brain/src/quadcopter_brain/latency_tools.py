class TemporalBuffer(object):
    def __init__(self, max_age=2.0):
        self.samples = []
        self.max_age = max_age

    def add(self, sample, timestamp):
        '''
        Add a timestamped sample to the buffer.
        '''
        # Remove any expired samples
        last_time = max(timestamp, self.samples[-1][0])
        while self.samples:
            (ts, _) = self.samples[0]
            if ts < last_time - self.max_age:
                self.samples.pop(0)
            else:
                break
        
        # Sanity check timestamp
        if timestamp < time.time() - self.max_age:
            return

        # Find insertion index
        idx = len(self.samples) - 1
        while idx >= 0:
            (ts, _) = self.samples[idx]
            if ts < timestamp:
                break
            idx -= 1

        # Insert sample
        self.samples.insert(idx + 1, (timestamp, sample))

    def query(self, timestamp, interpolate=True, extrapolate=False):
        '''
        Query the buffer at a given timestamp.
        '''
        # Check for out of temporal range
        if timestamp < self.samples[0][0]:
            raise IndexError(timestamp)
        if timestamp > self.samples[-1][0]:
            if not extrapolate:
                raise IndexError(timestamp)

            # If interpolation is not meaningful, the best we can do is
            # return the last sample read.
            if not interpolate:
                return self.samples[-1][1]

            # Use ballistic integration to estimate future position
            # First, find first and second derivative
            dx = [(t_0, (s_1 - s_0) / (t_1 - t_0)) for ((t_0, s_0), (t_1, s_1))
                   in zip(self.samples, self.samples[1:])]
            ddx = [(t_0, (s_1 - s_0) / (t_1 - t_0)) for ((t_0, s_0), (t_1, s_1))
                   in zip(dx, dx[1:])]

            # Average over the last three samples
            avg_dx = sum(dx[-3:]) / 3
            avg_ddx = sum(ddx[-3:]) / 3

            x0 = self.samples[-1][1]
            dt = timestamp - self.samples[-1][0]

            return x0 + dt * avg_dx + (dt**2 / 2) * avg_ddx


        # Find index of first sample ocurring after the requested time
        idx = 0
        while self.samples[idx][0] <= timestamp and idx < len(self.samples):
            idx += 1
        # Aaaand we want the one before that.
        idx -= 1

        if interpolate:
            # Linear interpolation
            idx_0, idx_1 = idx, idx + 1
            (t_0, s_0), (t_1, s_1) = self.samples[idx_0:idx_1]
            t = (timestamp - t_0) / float(t_1 - t_0)
            return s_0 + (s_1 - s_0) * t

        # First-order hold
        return self.samples[idx][1]
