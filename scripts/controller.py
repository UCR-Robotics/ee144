class Controller:
    def __init__(self, P=0.0, D=0.0, Derivator=0):
        self.Kp = P
        self.Kd = D
        self.Derivator = Derivator
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        # calculate P_value and D_value
        pass
        return P_value + D_value

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
    
    def setPD(self, set_P=0.0, set_D=0.0):
        self.Kp = set_P
        self.Kd = set_D