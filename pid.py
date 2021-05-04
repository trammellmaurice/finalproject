
import math
"""
PID CONTROLLER CLASS
"""

class pidController:
    def __init__(self,kp,ki=0,kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.errors = []

    def update(self, err):
        if len(self.errors) >= 5:
            self.errors.pop(0)
        self.errors.append(err)

    def p(self):
        return (self.kp * self.errors[len(self.errors)-1])

    def i(self):
        return (sum(self.errors) * self.ki)

    def d(self):
        return (self.kd * (self.errors[len(self.errors)-1]-self.errors[len(self.errors)-2]))

    def pid(self):
        return (self.p()+self.i()+self.d())
        #return (self.p())

if __name__ == '__main__':
    controller = pidController(0.5)
    controller.update(-1.51)
    controller.update(-1.21)
    print(controller.pid())
