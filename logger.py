from states import States

import logging
import os
from datetime import datetime
import numpy as np

class Logger:
    """Logging class to record system states, events, and debug information."""
    def __init__(self):
        # Create a logs directory
        os.makedirs("logs", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        # Main log file for events
        self.event_log = self._init_logger(f"logs/events_{timestamp}.log")
        # Separate log files for each robot (optional)
        self.robot_logs = {
            1: self._init_logger(f"logs/robot1_{timestamp}.log"),
            2: self._init_logger(f"logs/robot2_{timestamp}.log"),
            3: self._init_logger(f"logs/robot3_{timestamp}.log"),
            4: self._init_logger(f"logs/robot4_{timestamp}.log")
        }
        # Log file for state changes or other categories if needed
        self.state_log = self._init_logger(f"logs/state_{timestamp}.log")
    
    def _init_logger(self, filename):
        """Helper to initialize a logger that writes to the given filename."""
        logger = logging.getLogger(filename)
        logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(filename)
        fh.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        return logger
    
    def log_event(self, message):
        """Log a general event (info level)."""
        self.event_log.info(message)
        print(f"[EVENT] {message}")  # also print to console for immediate feedback
    
    def log_status(self, state: States, motions: list):
        """Log periodic status of robots and objects (e.g., joint angles, object positions)."""
        # Example: log each robot's joint positions and each pizza status
        status_msg = ""
        for i, motion in enumerate(motions, start=1):
            q = motion.robot.q
            status_msg += f"Robot{i} q={np.round(q, 3)}; "
            # Also log to each robot's individual log
            self.robot_logs[i].info(f"q={q}")
        for pizza in state.pizzas:
            status_msg += f"Pizza{pizza.id} stage={pizza.stage}, pos={np.round(pizza.mesh.pose.t,3)}; "
        self.state_log.info(status_msg)
    
    def log_error(self, message):
        """Log an error or critical issue."""
        self.event_log.error(message)
        print(f"[ERROR] {message}")
