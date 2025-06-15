class Interceptor:
    def __init__(self):
        # Current position
        self.pos_x = 0.0
        self.pos_y = 0.0
        
        # Current acceleration
        self.accel_x = 0.0
        self.accel_y = 0.0
        
        # Movement limits
        self.max_acceleration = 0.12

    def set_position(self, x, y):
        """Update current position"""
        self.pos_x = x
        self.pos_y = y

    def calculate_intercept_acceleration(self, target_x, target_y, time):
        """Calculate required acceleration to reach target in given time."""
        if time <= 0:
            return 0.0, 0.0
        
        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        
        ax = 2.0 * dx / (time * time)
        ay = 2.0 * dy / (time * time)
        
        return ax, ay

    def calculate_intercept(self, target_x, target_y, intercept_time):
        """Calculate interception parameters and return success status"""
        if intercept_time <= 0:
            print("Error: Intercept time must be positive")
            return False
            
        # Calculate required accelerations
        ax, ay = self.calculate_intercept_acceleration(target_x, target_y, intercept_time)
        
        # Check if accelerations are within reasonable bounds
        if abs(ax) > self.max_acceleration or abs(ay) > self.max_acceleration:
            print("Warning: Required acceleration too high")
            return False
        
        # Store calculated accelerations
        self.accel_x = ax
        self.accel_y = ay
        
        print(f"Calculated accelerations: ax={ax:.3f}, ay={ay:.3f}")
        return True

    def get_acceleration(self):
        """Return current acceleration values"""
        return self.accel_x, self.accel_y

    def reset(self):
        """Reset accelerations to zero"""
        self.accel_x = 0.0
        self.accel_y = 0.0