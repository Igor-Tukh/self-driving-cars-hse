from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.initial_yellow_count = None
        self.deviation_threshold = 2.0
        self.angle_step = 0.2
        self.forward_step = 1
        self.orientation = 0
    
    def see_duck(self, image):
        return np.less(self.deviation_threshold * self.initial_yellow_count,
                       self.count_yellow(image) - self.initial_yellow_count)

    def count_yellow(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        lower = np.array([20, 100, 100], dtype='uint8')
        upper = np.array([30, 255, 255], dtype='uint8')
        mask = np.array(cv2.inRange(hsv_img, lower, upper), dtype=np.bool)
        return mask.sum()
    
    def safe_ride_forward(self, env):
        while True:
            img = env.step([1, 0])[0]
            env.render()
            if self.see_duck(img):
                return
    
    def turn(self, env, steps, direction):
        img = None
        for _ in range(steps):
            img = env.step([0, self.angle_step * direction])[0]
            env.render()
            self.orientation -= direction
        return img
    
    def safe_turn_left(self, env):
        while True:
            img = self.turn(env, 1, 1)
            if not self.see_duck(img):
                break
                
    def small_ride_forward(self, env, steps=1):
        for _ in range(steps):
            env.step([1, 0])[0]
            env.render()
        
    def attempt_turn_right(self, env, steps):
        abort = False
        steps_count = 0
        for i in range(steps):
            img = self.turn(env, 1, -1)
            if self.see_duck(img):
                abort = True
                steps_count = i + 1
                break
        if abort:
            self.turn(env, steps_count, 1)
            return False
        return True
        
    def avoid_duck(self, env):
        self.safe_turn_left(env)
        rotation = np.abs(self.orientation)
        forward_steps = 0
        while True:
            self.small_ride_forward(env, self.forward_step)
            forward_steps += 1
            if self.attempt_turn_right(env, rotation):
                break
        while True:
            self.small_ride_forward(env)
            if self.attempt_turn_right(env, rotation):
                break
        self.small_ride_forward(env, forward_steps)
        self.turn(env, rotation, 1)
        self.small_ride_forward(env)
    
    def end_ride(self, env):
        env.step([0, 0])
        env.render()

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        self.initial_yellow_count = self.count_yellow(img)
        
        self.safe_ride_forward(env)
        self.avoid_duck(env)
        self.end_ride(env)
