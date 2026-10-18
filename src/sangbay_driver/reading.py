"""What every sensor type returns. No ROS."""


class Reading(object):
    """One measurement.

    concentration  float, in `unit`
    valid          False while preheating: the value is not to be used yet
    alarm          gas alarm (device alarm bits, or the configured threshold)
    fault          the sensor reports a problem (damaged, laser abnormal, ...)
    status         text: NORMAL, PREHEATING (12.3 s), HIGH, SENSOR-ERROR, ...
    extra          {name: float} published as extra topics (temperature, pressure, ...)
    values         [(key, text)] for diagnostics
    """

    def __init__(self, concentration, unit, gas, status='NORMAL', valid=True, alarm=False,
                 fault=False, extra=None, values=None):
        self.concentration = concentration
        self.unit = unit
        self.gas = gas
        self.status = status
        self.valid = valid
        self.alarm = alarm
        self.fault = fault
        self.extra = extra or {}
        self.values = values or []

    def apply_threshold(self, threshold):
        """Raise `alarm` at or above `threshold` (None: off)."""
        if threshold is not None and self.valid and self.concentration >= threshold:
            if not self.alarm:
                self.alarm = True
                self.status = 'THRESHOLD' if self.status == 'NORMAL' else self.status + ', THRESHOLD'
        return self

    def __repr__(self):
        return 'Reading(%s %g %s, %s)' % (self.gas, self.concentration, self.unit, self.status)
