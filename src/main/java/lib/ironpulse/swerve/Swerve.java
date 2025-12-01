package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import java.util.*;
import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
    private final SwerveConfig config;
    private final List<SwerveModule> modules;
    private final ImuIO imuIO;
    private final SwerveDriveKinematics kinematics;
    private final ImuIOInputsAutoLogged imuInputs;

    /**
     * Constructor: Initialize swerve drive with config and hardware interfaces
     * @param config Robot-specific configuration (dimensions, limits)
     * @param imuIO Gyro interface for robot rotation
     * @param moduleIOs Array of 4 module hardware interfaces (FL, FR, BL, BR)
     */
    public Swerve(SwerveConfig config, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        this.config = config;
        this.imuIO = imuIO;
        this.imuInputs = new ImuIOInputsAutoLogged();
        
        // Create 4 swerve modules
        this.modules = new ArrayList<>();
        for (int i = 0; i < config.moduleConfigs.length; i++)
            modules.add(new SwerveModule(config, config.moduleConfigs[i], moduleIOs[i]));
        
        // Setup kinematics (converts chassis speeds to module states)
        kinematics = new SwerveDriveKinematics(config.moduleLocations());
    }

    /**
     * Periodic: Called every 20ms by the robot loop
     * Updates sensor data and logs telemetry
     */
    @Override
    public void periodic() {
        // Update gyro inputs
        imuIO.updateInputs(imuInputs);
        Logger.processInputs(config.name + "/IMU", imuInputs);
        
        // Update all module inputs
        modules.forEach(m -> m.updateInputs());
    }

    /**
     * Main drive method: Convert desired chassis speeds to module commands
     * @param speeds Desired robot velocity (vx, vy, omega)
     */
    public void runTwist(ChassisSpeeds speeds) {
        // Apply velocity limits
        speeds = applyLimits(speeds);
        
        // Convert chassis speeds to individual module states
        var moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        // Prevent any wheel from exceeding max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates, 
            config.defaultSwerveLimit.maxLinearVelocity().in(MetersPerSecond)
        );
        
        // Command each module
        for (int i = 0; i < modules.size(); i++)
            modules.get(i).runState(moduleStates[i]);
    }
    
    /**
     * Stop all modules
     */
    public void runStop() {
        runTwist(new ChassisSpeeds());
    }

    /**
     * Apply velocity limits to chassis speeds
     */
    private ChassisSpeeds applyLimits(ChassisSpeeds speeds) {
        double maxLinear = config.defaultSwerveLimit.maxLinearVelocity().in(MetersPerSecond);
        double maxAngular = config.defaultSwerveLimit.maxAngularVelocity().in(RadiansPerSecond);
        
        double vx = Math.max(-maxLinear, Math.min(maxLinear, speeds.vxMetersPerSecond));
        double vy = Math.max(-maxLinear, Math.min(maxLinear, speeds.vyMetersPerSecond));
        double omega = Math.max(-maxAngular, Math.min(maxAngular, speeds.omegaRadiansPerSecond));
        
        return new ChassisSpeeds(vx, vy, omega);
    }
    
    /**
     * Get current robot rotation from gyro
     */
    public double getYawDegrees() {
        return imuInputs.yawPosition.getDegrees();
    }
    
    /**
     * Reset gyro angle to zero (for field-relative driving)
     */
    public void resetYaw() {
        imuIO.updateInputs(imuInputs);
    }
}