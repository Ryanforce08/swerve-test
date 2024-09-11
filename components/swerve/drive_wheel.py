from rev import CANSparkMax, SparkLimitSwitch, CANSparkLowLevel
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from wpimath.controller import PIDController
from phoenix6.controls.duty_cycle_out import DutyCycleOut


class DriveWheel:
    motor: TalonFX

    _target_speed: float
    _meters_per_rotation = 0.0532676904732978
    _min_velocity_to_engage_pid = 0.01

    def __init__(self, can_id):
        self._target_speed = 0
        # self.motor = CANSparkMax(can_id, CANSparkMax.MotorType.kBrushless)
        self.motor = TalonFX(can_id)
        self.config = TalonFXConfiguration()
        self.config.current_limits.supply_current_limit_enable = True
        self.config.current_limits.supply_current_limit = 60 # look into this more
        self.config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.pid=PIDController(0.1,0.1,0.1)
        # look at voltage compensation
        

      

        # self._motor_pid = self.motor.getPIDController()
        # self._motor_pid.setP(0.00001)
        # self._motor_pid.setI(0.000001)
        # self._motor_pid.setIZone(400)
        # self._motor_pid.setD(0.0)
        # self._motor_pid.setFF(0.00015)
        # self._motor_pid.setOutputRange(-1, 1)
        # # TODO: set status frame timing
        # self.motor.setSmartCurrentLimit(45)
        # self.motor.setSecondaryCurrentLimit(80)

        # self.motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        # self.motor.enableVoltageCompensation(12.0)
        # self.motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(False)
        # self.motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(False)

        # self.motor_encoder = self.motor.getEncoder()

    def get_current_velocity(self):
        # return self.motor_encoder.getVelocity() * DriveWheel._meters_per_rotation / 60.0 # convert to m/s from rpm
        return self.motor.get_velocity().value  * DriveWheel._meters_per_rotation

    def get_current_position(self):
        # AS: What's this used for?
        return self.motor.get_position().value * DriveWheel._meters_per_rotation

    def set_target_speed(self, target_speed):
        self._target_speed = target_speed

    def execute(self):
        if abs(self._target_speed) < DriveWheel._min_velocity_to_engage_pid:
            # set raw power
            
            self.motor.set_control(DutyCycleOut(0))
        else:
            target_speed_rpm = self._target_speed / DriveWheel._meters_per_rotation
            output = self.pid.calculate(self.get_current_velocity(), target_speed_rpm) 
            self.motor.set_control(DutyCycleOut(output))
        
