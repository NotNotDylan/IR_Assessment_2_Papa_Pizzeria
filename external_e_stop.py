import serial  # if using pyserial for Arduino comm (ensure to install pyserial)

class ExternalEStop:
    """Handles input from a physical emergency stop button (via Arduino) asynchronously."""
    def __init__(self, port: str = None):
        # port: specify the serial port of Arduino if applicable
        self.is_pressed = False  # True if the physical e-stop is currently pressed
        # If using serial communication:
        self.serial = None
        if port:
            try:
                self.serial = serial.Serial(port, 9600, timeout=1)
            except Exception as e:
                print(f"Warning: Could not open serial port for E-Stop: {e}")
        # You might start a background thread to continuously listen for Arduino signals
        # Example: threading.Thread(target=self._listen_serial, daemon=True).start()
    
    def _listen_serial(self):
        """Background thread target: listen to serial for e-stop signals."""
        if not self.serial:
            return
        while True:
            try:
                line = self.serial.readline().decode().strip()
                # Assume the Arduino sends "ESTOP\n" when pressed and "CLEAR\n" when released, for example
                if line == "ESTOP":
                    self.is_pressed = True
                elif line == "CLEAR":
                    self.is_pressed = False
            except Exception as e:
                print(f"E-Stop serial read error: {e}")
                break
    
    def check_stop(self) -> bool:
        """Check if an emergency stop has been triggered (either hardware or via GUI)."""
        # This method can be called in each loop iteration.
        # If hardware e-stop is pressed, this returns True.
        # In a real scenario, this could also incorporate other emergency conditions.
        return self.is_pressed

    def reset(self):
        """Reset the E-stop state (after the button is released and system is confirmed to resume)."""
        # Possibly send an acknowledge to Arduino or just set flag false.
        self.is_pressed = False
