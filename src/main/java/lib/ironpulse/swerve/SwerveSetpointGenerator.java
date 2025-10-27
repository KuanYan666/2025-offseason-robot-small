package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.*;
import lombok.*;
import static edu.wpi.first.units.Units.*;

@Builder
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;
    @Getter @Setter private SwerveLimit chassisLimit;
    @Getter @Setter private SwerveModuleLimit moduleLimit;

    public SwerveSetpoint generate(ChassisSpeeds desired, SwerveSetpoint prev, double dt) {
        // Apply limits
        desired = chassisLimit.apply(prev.chassisSpeeds(), desired, dt);
        
        // Convert to module states
        var states = kinematics.toSwerveModuleStates(desired);
        
        // Desaturate wheels
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 
            moduleLimit.maxDriveVelocity().in(MetersPerSecond));
        
        // Keep previous angles if stopped
        if (Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond) < 0.001)
            for (int i = 0; i < states.length; i++)
                states[i].angle = prev.moduleStates()[i].angle;
        
        return new SwerveSetpoint(desired, states);
    }
}