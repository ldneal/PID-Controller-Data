import time

# PID controller class
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired setpoint
        self.prev_error = 0  # Previous error
        self.integral = 0  # Integral term
        
    def compute(self, current_value, dt):
        # Calculate the error
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        
        # Calculate the output
        output = P + I + D
        
        # Save the error for the next loop
        self.prev_error = error
        
        return output

# Simulated system
def system_simulation(input_value, system_state):
    # Simple model for system dynamics
    response = system_state + (input_value * 0.1)
    return response

# Main function
if __name__ == "__main__":
    setpoint = 100  # Target value for the system (e.g., temperature in degrees)
    
    # PID controller parameters (tune these for your system)
    Kp = 2.0
    Ki = 0.5
    Kd = 1.0
    
    pid = PID(Kp, Ki, Kd, setpoint)
    
    # Simulation parameters
    system_state = 20.0  # Initial value of the system (e.g., room temperature)
    dt = 0.1  # Time step in seconds
    time_duration = 20  # Duration of simulation in seconds
    
    # Simulation loop
    print("Time (s)\tSystem State\tControl Output")
    for t in range(int(time_duration / dt)):
        current_time = t * dt
        
        # Get the control output from the PID controller
        control_output = pid.compute(system_state, dt)
        
        # Simulate the system response
        system_state = system_simulation(control_output, system_state)
        
        # Display the system state and control output
        print(f"{current_time:.1f}\t\t{system_state:.2f}\t\t{control_output:.2f}")
        
        # Simulate real-time processing (you can remove this in real implementations)
        time.sleep(dt)
