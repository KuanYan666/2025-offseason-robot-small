package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.ironpulse.swerve.ImuIO;

/**
 * Simplified Pigeon2 Gyro Implementation
 * Reads yaw (robot rotation) for field-relative driving
 */
public class ImuIOPigeon implements ImuIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    /**
     * Initialize Pigeon2 gyro
     * @param config Swerve configuration containing gyro ID and CAN bus name
     */
    public ImuIOPigeon(SwerveSJTU6Config config) {
        // Initialize Pigeon2 on CANivore bus
        pigeon = new Pigeon2(config.pigeonId, config.canivoreCanBusName);
        
        // Get yaw signals (rotation around vertical axis)
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        
        // Configure update rate: 50Hz for basic operation
        yaw.setUpdateFrequency(50.0);
        yawVelocity.setUpdateFrequency(50.0);
        
        pigeon.optimizeBusUtilization();
    }

    /**
     * Update gyro sensor data
     * Refreshes signals and reads current values
     */
    @Override
    public void updateInputs(ImuIOInputs inputs) {
        // Refresh signals from hardware
        BaseStatusSignal.refreshAll(yaw, yawVelocity);
        
        // Check connection status
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        
        // Read yaw (robot rotation)
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(Units.RadiansPerSecond);
    }
}