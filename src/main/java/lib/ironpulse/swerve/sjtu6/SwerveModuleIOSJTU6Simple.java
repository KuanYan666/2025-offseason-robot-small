package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveModuleIO;

import static edu.wpi.first.units.Units.*;

/**
 * Simplified CTRE Phoenix 6 Swerve Module Implementation
 * Controls one swerve module using TalonFX motors
 */
public class SwerveModuleIOSJTU6Simple implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;
    private final SwerveSJTU6Config config;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    
    // Control requests
    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final PositionDutyCycle steerPositionRequest = new PositionDutyCycle(0);
    
    // Status signals
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveVoltage;
    private final StatusSignal<Current> driveCurrentAmps;
    
    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Voltage> steerVoltage;
    private final StatusSignal<Current> steerCurrentAmps;

    /**
     * Initialize swerve module hardware
     * @param config Overall swerve configuration
     * @param moduleIndex Index of this module (0-3)
     */
    public SwerveModuleIOSJTU6Simple(SwerveSJTU6Config config, int moduleIndex) {
        this.config = config;
        this.moduleConfig = config.moduleConfigs[moduleIndex];
        
        // Create motor and encoder objects
        driveMotor = new TalonFX(moduleConfig.driveMotorId, config.canivoreCanBusName);
        steerMotor = new TalonFX(moduleConfig.steerMotorId, config.canivoreCanBusName);
        encoder = new CANcoder(moduleConfig.encoderId, config.canivoreCanBusName);
        
        // Configure motors
        configureDriveMotor();
        configureSteerMotor();
        
        // Get status signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrentAmps = driveMotor.getSupplyCurrent();
        
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();
        steerVoltage = steerMotor.getMotorVoltage();
        steerCurrentAmps = steerMotor.getSupplyCurrent();
        
        // Set update frequencies (50Hz for simplified version)
        drivePosition.setUpdateFrequency(50.0);
        driveVelocity.setUpdateFrequency(50.0);
        driveVoltage.setUpdateFrequency(50.0);
        driveCurrentAmps.setUpdateFrequency(50.0);
        
        steerPosition.setUpdateFrequency(50.0);
        steerVelocity.setUpdateFrequency(50.0);
        steerVoltage.setUpdateFrequency(50.0);
        steerCurrentAmps.setUpdateFrequency(50.0);
        
        // Optimize CAN bus usage
        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    /**
     * Configure drive motor (wheel)
     */
    private void configureDriveMotor() {
        var config = new TalonFXConfiguration();
        
        // Motor direction
        config.MotorOutput.Inverted = moduleConfig.driveInverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = this.config.driveStatorCurrentLimit.in(Amp);
        
        // PID gains (will be set later via config calls)
        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0.16;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.1247;
        config.Slot0.kA = 0.01215;
        
        driveMotor.getConfigurator().apply(config);
    }

    /**
     * Configure steer motor (module rotation)
     */
    private void configureSteerMotor() {
        var motorConfig = new TalonFXConfiguration();
        var encoderConfig = new CANcoderConfiguration();
        
        // Encoder configuration
        encoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderInverted ? 
            SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = moduleConfig.steerMotorEncoderOffset.magnitude();
        
        // Motor direction
        motorConfig.MotorOutput.Inverted = moduleConfig.steerInverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Use CANcoder for absolute position feedback
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId;
        motorConfig.Feedback.RotorToSensorRatio = this.config.steerGearRatio;
        
        // Current limits
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = this.config.steerStatorCurrentLimit.in(Amp);
        
        // PID gains
        motorConfig.Slot0.kP = 10.0;
        motorConfig.Slot0.kI = 0.001;
        motorConfig.Slot0.kD = 0.15;
        motorConfig.Slot0.kS = 0.005;
        
        // Enable continuous wrap (module can spin continuously)
        motorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        encoder.getConfigurator().apply(encoderConfig);
        steerMotor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Refresh all signals from hardware
        BaseStatusSignal.refreshAll(
            drivePosition, driveVelocity, driveVoltage, driveCurrentAmps,
            steerPosition, steerVelocity, steerVoltage, steerCurrentAmps
        );
        
        // Drive motor data
        inputs.driveMotorConnected = BaseStatusSignal.isAllGood(drivePosition, driveVelocity);
        inputs.driveMotorPositionRad = driveMotorRotationsToWheelRad(drivePosition.getValueAsDouble());
        inputs.driveMotorVelocityRadPerSec = driveMotorRotationsToWheelRad(driveVelocity.getValueAsDouble());
        inputs.driveMotorVoltageVolt = driveVoltage.getValueAsDouble();
        inputs.driveMotorCurrentAmpere = driveCurrentAmps.getValueAsDouble();
        
        // Steer motor data
        inputs.steerMotorConnected = BaseStatusSignal.isAllGood(steerPosition, steerVelocity);
        inputs.steerMotorPositionRad = Units.rotationsToRadians(steerPosition.getValueAsDouble());
        inputs.steerMotorVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerMotorVoltageVolt = steerVoltage.getValueAsDouble();
        inputs.steerMotorCurrentAmpere = steerCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity) {
        // Convert m/s to motor rotations per second
        double wheelRPS = velocity.in(MetersPerSecond) / (config.wheelDiameter.in(Meter) * Math.PI);
        double motorRPS = wheelRPS * config.driveGearRatio;
        driveMotor.setControl(driveVelocityRequest.withVelocity(motorRPS));
    }

    @Override
    public void setSteerAngleAbsolute(Angle angle) {
        // Convert radians to motor rotations
        double motorRotations = Units.radiansToRotations(angle.in(Radian));
        steerMotor.setControl(steerPositionRequest.withPosition(motorRotations));
    }

    // ========== UNIT CONVERSION ==========
    
    /**
     * Convert drive motor rotations to wheel radians
     * Motor rotates faster than wheel due to gear ratio
     */
    private double driveMotorRotationsToWheelRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations) / config.driveGearRatio;
    }
}
