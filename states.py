from enum import Enum, auto

class SystemState(Enum):
    # For operations
    READY = auto()      # Waiting to be run
    RUNNING = auto()    # Currently executing
    BLOCKED = auto()    # Waiting (e.g. delay, waiting on semaphore, sleep)
    ACHIEVED = auto()   # Operation has run and has finished
    SUSPENDED = auto()  # Explicitly paused by other task or API
    IDLE = auto()       # Does nothing
    
    
    # For inputs
    ACTIVE = auto()     # e.g. button pressed, sensor tripped
    DEACTIVE = auto()   # e.g. not pressed, not tripped

class State:
    """
    Tracks the state of any system or object.
    """
    _instances = []  # class attribute to hold all instances
    
    def __init__(self, state: SystemState | None = None):
        self.state = state
        self.next_state = state
        
        self.next_state_priority = 0
        
        if self.state == None:
            self.state = SystemState.IDLE
        
        State._instances.append(self)
        
    # Set states
    def set_state(self, new_state: SystemState):
        """Overrids and sets current state"""
        self.state = new_state
    
    def set_next_state(self, new_state: SystemState, priority: int = 0):
        """ Sets next state after update, priority has to be > and != than previous set priority to override """
        
        if priority > self.next_state_priority:
            self.next_state = new_state
            self.next_state_priority = priority
            
    def lock_next_state(self, new_state: SystemState):
        """If called next state will be what's called here"""
        self.next_state = new_state
        self.next_state_priority = 9999 # Set higher than it is reasonably to be set anywhere else in code

    # Checking current state
    def is_running(self) -> bool:
        return self.state is SystemState.RUNNING
    
    def is_ready(self) -> bool:
        return self.state is SystemState.READY

    def is_idle(self) -> bool:
        return self.state is SystemState.IDLE
    
    def is_achieved(self) -> bool:
        return self.state is SystemState.ACHIEVED
    
    def is_suspended(self) -> bool:
        return self.state is SystemState.SUSPENDED
    
    def is_active(self) -> bool:
        return self.state is SystemState.ACTIVE
    
    def will_be_running(self) -> bool:
        return self.next_state is SystemState.RUNNING

    def will_be_idle(self) -> bool:
        return self.next_state is SystemState.IDLE

    # Updates states at next cycle
    def update(self):
        """Set state = next_state, next_state remains unchanged but priority is reset to 0"""
        self.state = self.next_state
        self.next_state_priority = 0

    @classmethod # only need to call once and will update all instances of this class
    def step_all(cls):
        for inst in cls._instances:
            inst.update()
    
