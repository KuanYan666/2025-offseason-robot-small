package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for the gyroscope (IMU)
 * Tracks robot rotation for field-relative driving
 */
public interface ImuIO {
    /**
     * Update gyro sensor data
     */
    default void updateInputs(ImuIOInputs inputs) {
    }

    /**
     * Gyro sensor data
     * Only tracks yaw (rotation around vertical axis)
     */
    @AutoLog
    class ImuIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();  // Current robot rotation
        public double yawVelocityRadPerSec = 0.0;         // Rotation speed
    }
}
