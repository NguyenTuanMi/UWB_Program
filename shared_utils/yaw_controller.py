import time
from typing import Optional

class YawController:
    """
    A blocking controller for precise angle-based yaw rotation using RC velocity commands.
    """
    
    DEFAULT_TIMEOUT_SEC = 10.0
    DEFAULT_TOLERANCE_DEG = 3.0
    DEFAULT_YAW_SPEED = 60  # RC velocity command (0-100)
    SLOWDOWN_THRESHOLD_DEG = 30.0  # Start slowing down within this range
    MIN_YAW_SPEED = 20  # Minimum RC yaw velocity during slowdown
    
    def __init__(self, controller):
        """
        Args:
            controller: Instance of DroneController
        """
        self.controller = controller
    
    def yaw_right_by_angle(
            self, 
            angle: float, 
            speed=DEFAULT_YAW_SPEED, 
            timeout=DEFAULT_TIMEOUT_SEC, 
            tolerance=DEFAULT_TOLERANCE_DEG
        ):
        """Rotate clockwise (right) by specified angle and block until complete."""
        self._execute_rotation(
            abs(angle), 
            direction=1, 
            speed=speed, 
            timeout=timeout, 
            tolerance=tolerance
        )
    
    def yaw_left_by_angle(
            self, 
            angle: float, 
            speed=DEFAULT_YAW_SPEED, 
            timeout=DEFAULT_TIMEOUT_SEC, 
            tolerance=DEFAULT_TOLERANCE_DEG
        ):
        """Rotate counter-clockwise (left) by specified angle and block until complete."""
        self._execute_rotation(
            abs(angle), 
            direction=-1, 
            speed=speed, 
            timeout=timeout, 
            tolerance=tolerance
        )
    
    def _execute_rotation(
            self, 
            target_angle: float, 
            direction: int, 
            speed: int, 
            timeout: float, 
            tolerance: float
        ):
        """Internal blocking loop to execute the rotation."""
        start_time = time.time()
        accumulated_rotation = 0.0
        
        # 1. Get initial heading
        last_heading = self._get_safe_heading()
        if last_heading is None:
            print("[YAW CONTROLLER] Failed to get initial heading.")
            return
            
        print(f"[YAW CONTROLLER] Starting rotation. Target: {target_angle}°, Direction: {'CW' if direction > 0 else 'CCW'}")

        while True:
            # Abort quickly if mission/controller is stopping.
            if not getattr(self.controller, "is_running", True):
                print("[YAW CONTROLLER] Aborted: controller is not running.")
                break
            
            # 2. Check Timeout
            if (time.time() - start_time) > timeout:
                print(f"[YAW CONTROLLER] Warning: Rotation timed out after {timeout}s.")
                break
                
            # 3. Update Heading
            current_heading = self._get_safe_heading()
            if current_heading is None:
                time.sleep(0.05)
                continue

            # 4. Calculate Delta and Accumulate
            accumulated_rotation += self._calculate_progress(current_heading, last_heading, direction)
            last_heading = current_heading
            remaining_angle = target_angle - accumulated_rotation
            
            # 5. Check Completion
            if remaining_angle <= tolerance:
                print(f"[YAW CONTROLLER] Rotation complete. Accumulated: {accumulated_rotation:.1f}°")
                break
                
            # 6. Calculate Adaptive Speed & Send Command
            current_speed = self._calculate_adaptive_speed(speed, remaining_angle)
            self.controller.drone.send_rc_control(0, 0, 0, int(current_speed * direction))            
            time.sleep(0.2)
            
        # 7. Stop Rotation
        self._stop_rotation()

    def _get_safe_heading(self) -> Optional[float]:
        """Attempts to retrieve the current heading safely."""
        try:
            return float(self.controller.get_heading())
        except Exception:
            return None

    def _calculate_progress(self, current_heading: float, last_heading: float, direction: int) -> float:
        """Calculates angular progress made in the intended direction."""
        delta = current_heading - last_heading
        
        # Normalize delta to [-180, 180] mapping
        while delta > 180: delta -= 360
        while delta <= -180: delta += 360
        
        # CW progress yields positive delta, CCW yields negative delta
        progress = delta * direction
        return progress if progress > 0 else 0.0

    def _calculate_adaptive_speed(self, base_speed: int, remaining_angle: float) -> int:
        """Calculates yaw speed, applying linear deceleration near the target."""
        if remaining_angle >= self.SLOWDOWN_THRESHOLD_DEG:
            return base_speed
            
        ratio = remaining_angle / self.SLOWDOWN_THRESHOLD_DEG
        return max(self.MIN_YAW_SPEED, int(base_speed * ratio))

    def _stop_rotation(self):
        """Sends multiple stop commands to ensure reliability."""
        for _ in range(3):
            time.sleep(0.1)
            self.controller.drone.send_rc_control(0, 0, 0, 0)