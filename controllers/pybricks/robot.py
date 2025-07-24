from controller import Robot

class RobotSingleton:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize_robot()
        return cls._instance
    
    def _initialize_robot(self):
       self.time_step = 32
       
       try:
           self.robot = Robot()
       except Exception as e:
           raise RuntimeError("Error initializing robot: " + str(e))

    def get_robot(self):
        if hasattr(self, "robot"):
            return self.robot
        else:
            raise RuntimeError("Robot instance not available.")
    
    def get_time_step(self):
        return self.time_step
