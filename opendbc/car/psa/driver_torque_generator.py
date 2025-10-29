import numpy as np
import random



class DriverTorqueGenerator:
    """
    Generates a simulated driver torque
    """
    def __init__(self):
        self.start_curve = random.randint(0, 10)
        self.end_curve =  random.randint(11, 30)
        self.actual_value = self.start_curve
        self.sign = +1


    def next_value(self):
        self.actual_value += random.randint(0, 10) /10 * self.sign
        if self.actual_value >=  self.end_curve:
            self.sign = -1
            self.start_curve = random.randint(0, 10)
        if self.actual_value <=  self.start_curve:
            self.sign = +1
            self.end_curve =  random.randint(11, 30)

        return self.actual_value