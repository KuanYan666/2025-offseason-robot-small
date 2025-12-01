package lib.ironpulse.swerve;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for a swerve module
 * Defines methods for reading sensors and controlling motors
 */
public interface SwerveModuleIO {
    /**
     * Update sensor inputs from hardware
     */
    default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    /**
     * Set drive motor velocity (closed-loop control)
     */
    default void setDriveVelocity(LinearVelocity desiredVelocity) {
    }

    /**
     * Set steer motor angle (closed-loop control)
     */
    default void setSteerAngleAbsolute(Angle desiredAngle) {
    }


    /**
     * Sensor data from a swerve module
     * AutoLog annotation automatically generates logging code
     */
    @AutoLog
    class SwerveModuleIOInputs {
        // Drive motor (wheel)
        public boolean driveMotorConnected = false;
        public double driveMotorPositionRad = 0.0;
        public double driveMotorVelocityRadPerSec = 0.0;
        public double driveMotorVoltageVolt = 0.0;
        public double driveMotorCurrentAmpere = 0.0;

        // Steer motor (module rotation)
        public boolean steerMotorConnected = false;
        public double steerMotorPositionRad = 0.0;
        public double steerMotorVelocityRadPerSec = 0.0;
        public double steerMotorVoltageVolt = 0.0;
        public double steerMotorCurrentAmpere = 0.0;
    }
}
