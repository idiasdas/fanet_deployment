from import_file import *


class Trace:
    def __init__(self, n_targets, observation_period, target_speed, area_size, time_step_delta=1):
        """Represents a set of targets moving on a xy plane with the random way point model. The targets are moving at a constant speed and bounce off the walls of the square area A = (area_size x area_size). The trace is a sequence of positions of the targets at each time step. The trace is generated at initialization.

        Args:
            n_targets (int): Amount of targets
            observation_period (int): Number of time steps
            target_speed (float): Targets speed in m/s
            area_size (float): Lenght of the square area A
            time_step_delta (float, optional): Amount of seconds between time steps. Defaults to 1.
        """
        self.n_targets = n_targets                      
        self.observation_period = observation_period    
        self.target_speed = target_speed                
        self.area_size = area_size                      
        self.time_step_delta = time_step_delta          

    def generate_random_direction(self):
        """Generates a random normalized vector of dimension 2, returns as a tuple."""
        direction = (np.random.rand(2) - 0.5)*2
        normalized_direction = tuple(direction/np.linalg.norm(direction))
        return normalized_direction

