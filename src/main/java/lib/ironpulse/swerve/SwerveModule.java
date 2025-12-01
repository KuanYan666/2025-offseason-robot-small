package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

/**
 * Simplified Swerve Module
 * Controls one corner of the robot (drive + steer motors)
 */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged data;
    private final SwerveConfig config;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;

    /**
     * Constructor: Create a swerve module
     * @param config Overall swerve configuration
     * @param moduleConfig This specific module's config
     * @param io Hardware interface for this module
     */
    public SwerveModule(SwerveConfig config, SwerveConfig.SwerveModuleConfig moduleConfig, SwerveModuleIO io) {
        this.io = io;
        this.config = config;
        this.moduleConfig = moduleConfig;
        this.data = new SwerveModuleIOInputsAutoLogged();
    }

    /**
     * Update sensor data from hardware
     */
    public void updateInputs() {
        io.updateInputs(data);
        Logger.processInputs(config.name + "/Module/" + moduleConfig.name, data);
    }

    /**
     * Command the module to a desired state
     * @param state Desired wheel speed (m/s) and angle (rotation)
     */
    public void runState(SwerveModuleState state) {
        io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }
}