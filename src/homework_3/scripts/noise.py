import time
import numpy as np
import matplotlib.pyplot as plt

class Noise():
    def __init__(self):
        self.noise = []
        self.time = []
        self.duration = 10
        self.mu, self.sigma = 0, 0.01 
            
    def noise_sim(self):
        prev_time = time.time()
        current_time = prev_time
        while(current_time - prev_time < self.duration):
                noise = np.random.normal(self.mu, self.sigma)
                self.time.append(current_time - prev_time)
                self.noise.append(noise)
                current_time = time.time()
                time.sleep(0.001)
        self.visualize_noise()
    
    def visualize_noise(self):
        plt.plot(self.time, self.noise, c='r')
        plt.xlabel('Time')
        plt.ylabel('Noise')
        plt.title(f'Noise vs Time (mu={self.mu}, sigma={self.sigma})')
        plt.show()

if __name__ == "__main__":
    noise = Noise()
    noise.noise_sim()