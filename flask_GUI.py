from flask import Flask, render_template, request

class FlaskGUI:
    """Flask-based web GUI for controlling the simulation (e-stop, manual teach, etc.)."""
    def __init__(self):
        self.app = Flask(__name__)
        self.estop_pressed = False    # True if E-stop button pressed (simulated via UI)
        self.resume_requested = False # True if resume button pressed after an e-stop
        self.stop_pressed = False     # True if a general stop/quit button is pressed
        self.manual_mode_robot = None # Which robot is currently selected for manual control (1-4 or None)
        # Additional state as needed (e.g., slider values for joints, target positions, etc.)
        
        # Set up Flask routes
        @self.app.route('/')
        def index():
            # Render main control page (you will create an HTML template with buttons/controls)
            return render_template('index.html')
        
        @self.app.route('/estop', methods=['POST'])
        def estop_button():
            # Route triggered when E-stop button is pressed on GUI
            self.estop_pressed = True
            # (In a real scenario, you'd also physically press the hardware e-stop)
            return ("", 204)
        
        @self.app.route('/resume', methods=['POST'])
        def resume_button():
            # Route for resume button (to be pressed after e-stop is released and ready to continue)
            if not self.estop_pressed:
                # Only allow resume if e-stop state is cleared
                self.resume_requested = True
            return ("", 204)
        
        @self.app.route('/select_robot', methods=['POST'])
        def select_robot():
            # Route to select a robot for manual control (e.g., via dropdown or buttons)
            robot_id = request.form.get('robot_id')  # assume the form sends an id
            if robot_id is not None:
                self.manual_mode_robot = int(robot_id)
            return ("", 204)
        
        @self.app.route('/joint_control', methods=['POST'])
        def joint_control():
            # Route to handle manual joint control from sliders.
            # It should read which joint and value from request, then command the corresponding robot.
            robot_id = int(request.form.get('robot_id'))
            joint_index = int(request.form.get('joint_index'))
            joint_value = float(request.form.get('joint_value'))
            # TODO: Implement: send the joint value to the specified robot (e.g., update robot model joint or store the command)
            # Possibly, you'll interface with the Robot movement classes or directly with the robot model.
            return ("", 204)
        
        # Additional routes can be defined for Cartesian control, flail arms demo, etc.
    
    def start_server_async(self, host='0.0.0.0', port=5000):
        """Start the Flask web server in a separate thread so the main program can continue."""
        # NOTE: This is a simple approach. Flask's built-in server is not ideal for production but fine for this simulation.
        
        # thread = threading.Thread(target=self.app.run, kwargs={'host': host, 'port': port, 'debug': False, 'use_reloader': False})
        # thread.daemon = True  # Daemonize thread so it won't prevent program exit
        # thread.start()
