"""STUDENT FILE. Implement the search state machine. No ROS calls in this module.
Use raster() and drive_to(); never read simulator internals or web /state.
Return Outcome('absent') ONLY after covering the entire requested area.
Return Outcome('found', x, y) after measuring the rectangle centre.
"""
from course_lab.world import Area, Sample, Decision
from course_lab.navigation import drive_to, raster
from course_lab.probe import RectangleProbe
from course_lab.world import Outcome

class Mission:
    def __init__(self, area: Area):
        self.area = area
        self.waypoints = raster(area)
        self.gift_measurement = None
        self.index = 0

    def step(self, sample: Sample) -> Decision:
        # TODO M1: follow raster; on green INSIDE area instantiate RectangleProbe.
        # Probe already measures boundaries. Delegate future samples to probe.step().
        # Basic feedback and all safety/failure policy are supplied by the adapter.
        # Starter is intentionally stopped, not a working submission.

        if self.gift_measurement is None:
            if sample.green and self.area.contains(sample.x, sample.y):
                self.gift_measurement = RectangleProbe(self.area, sample)
                return self.gift_measurement.step(sample)
            else:
                v,w,dest_flag = drive_to(sample, self.waypoints[self.index])
                if dest_flag:
                    self.index += 1
                if self.index >= len(self.waypoints):
                    return Decision(v=0.0, w=0.0, outcome=Outcome('absent'))
                else:
                    return Decision(v, w)
        else:
            return self.gift_measurement.step(sample)
