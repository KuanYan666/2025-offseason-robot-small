package lib.ironpulse.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import static edu.wpi.first.units.Units.*;

@Builder
public record SwerveLimit(
    LinearVelocity maxLinearVelocity, 
    LinearAcceleration maxSkidAcceleration,
    AngularVelocity maxAngularVelocity, 
    AngularAcceleration maxAngularAcceleration
) {
    public ChassisSpeeds apply(ChassisSpeeds curr, ChassisSpeeds des, double dt) {
        // Simple velocity clamping only
        double vx = MathUtil.clamp(des.vxMetersPerSecond, 
            -maxLinearVelocity.in(MetersPerSecond), maxLinearVelocity.in(MetersPerSecond));

        double vy = MathUtil.clamp(des.vyMetersPerSecond, 
            -maxLinearVelocity.in(MetersPerSecond), maxLinearVelocity.in(MetersPerSecond));

        double omega = MathUtil.clamp(des.omegaRadiansPerSecond,
            -maxAngularVelocity.in(RadiansPerSecond), maxAngularVelocity.in(RadiansPerSecond));
            
        return new ChassisSpeeds(vx, vy, omega);
    }
}