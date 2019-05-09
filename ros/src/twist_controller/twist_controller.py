import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio,
                max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Controller Parameter
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # min throttle
        mx = 0.2 # max throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1 / (2 * PI * tau) = cutoff frequency
        ts = .02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        #[1] reset (prevent error accumulation) and send 0 for outputs if DBW is not enabled
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 1.0, 0.0, 0.0 # Default for what you want the car to do (good for debugging)
        
        # [2] filter out noise from input velocity
        current_vel = self.vel_lpf.filt(current_vel)

        # [3] apply yaw controller to get steering
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # [4] linear velocity error
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # [5] calculate sample time for PID
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # [6] throttle pid
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # if goal is to stop and we are slow, apply break to hold car
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 # Nm - to hold the car in place if we are stopped at a light.

        # slowing down proportionally 
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque in Nm

        return throttle, brake, steering
