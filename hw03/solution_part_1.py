from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.initial_yellow_count = None
        self.deviation_threshold = 5.0

    def see_duck(self, image):
        return np.less(self.deviation_threshold * self.initial_yellow_count,
                       self.count_yellow(image) - self.initial_yellow_count)

    def count_yellow(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        lower = np.array([20, 100, 100], dtype='uint8')
        upper = np.array([30, 255, 255], dtype='uint8')
        mask = np.array(cv2.inRange(hsv_img, lower, upper), dtype=np.bool)
        return mask.sum()

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        self.initial_yellow_count = self.count_yellow(img)
        while True:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            # add here some image processing
            env.render()
            if self.see_duck(img):
                env.step([0, 0])
                env.render()
                break
