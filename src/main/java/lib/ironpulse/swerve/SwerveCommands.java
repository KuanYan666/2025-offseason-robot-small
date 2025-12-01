package lib.ironpulse.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Simplified Swerve Commands
 * Factory methods for creating swerve drive commands
 */
public class SwerveCommands {


    /**
     * Create joystick drive command (field-relative)
     * @param swerve Swerve subsystem
     * @param xSupplier Forward/backward joystick (-1 to 1)
     * @param ySupplier Left/right joystick (-1 to 1)
     * @param zSupplier Rotation joystick (-1 to 1)
     * @param robotAngleSupplier Current robot angle (for field-relative)
     * @param translationDeadband Minimum translation input
     * @param rotationDeadband Minimum rotation input
     * @return Command that drives the robot
     */
    public static Command driveWithJoystick(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> robotAngleSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband
    ) {
        return Commands.run(() -> {
            // Read joystick inputs
            double xInput = xSupplier.getAsDouble();
            double yInput = ySupplier.getAsDouble();
            double zInput = zSupplier.getAsDouble();
            
            // Apply deadband to avoid drift
            double xSpeed = MathUtil.applyDeadband(xInput, 0.05);
            double ySpeed = MathUtil.applyDeadband(yInput, 0.05);
            double zSpeed = MathUtil.applyDeadband(zInput, 0.05);
            
            // Apply quadratic curve for finer control at low speeds
            xSpeed = xSpeed * Math.abs(xSpeed);  // Preserve sign
            ySpeed = ySpeed * Math.abs(ySpeed);
            zSpeed = zSpeed * Math.abs(zSpeed);
            
            // Scale to max speeds (4.5 m/s linear, 10 rad/s rotation)
            xSpeed *= 4.5;
            ySpeed *= 4.5;
            zSpeed *= 10.0;
            
            // Convert to field-relative speeds
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, zSpeed,
                    robotAngleSupplier.get().getRotation().toRotation2d()
            );
            
            swerve.runTwist(chassisSpeeds);
        }, swerve);
    }

    /**
     * Create command to stop the robot
     */
    public static Command stop(Swerve swerve) {
        return Commands.runOnce(swerve::runStop, swerve);
    }
    
    /**
     * Create command to reset gyro angle to zero
     */
    public static Command resetAngle(Swerve swerve, Rotation2d angle) {
        return Commands.runOnce(swerve::resetYaw).ignoringDisable(true);
    }
}
