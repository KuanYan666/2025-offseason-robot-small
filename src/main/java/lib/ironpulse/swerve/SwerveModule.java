package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.*;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged data;
    private final SwerveConfig config;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    @Getter private SwerveModulePosition[] odometryPositions;

    public SwerveModule(SwerveConfig config, SwerveConfig.SwerveModuleConfig moduleConfig, SwerveModuleIO io) {
        this.io = io;
        this.config = config;
        this.moduleConfig = moduleConfig;
        this.data = new SwerveModuleIOInputsAutoLogged();
    }

    public void updateInputs() {
        io.updateInputs(data);
        Logger.processInputs(config.name + "/Module/" + moduleConfig.name, data);
    }

    public void periodic() {
        int n = Math.min(data.driveMotorPositionRadSamples.length, data.steerMotorPositionRadSamples.length);
        odometryPositions = new SwerveModulePosition[n];
        for (int i = 0; i < n; i++)
            odometryPositions[i] = new SwerveModulePosition(
                data.driveMotorPositionRadSamples[i] * config.wheelDiameter.in(Meter) * 0.5,
                new Rotation2d(data.steerMotorPositionRadSamples[i]));
    }

    public void runState(SwerveModuleState state) {
        io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }

    public void runState(SwerveModuleState state, Current ff) {
        io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond), ff);
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }

    public void runDriveVoltage(Voltage voltage) {
        io.setDriveOpenLoop(voltage);
        io.setSteerAngleAbsolute(Rotation2d.kZero.getMeasure());
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            data.driveMotorVelocityRadPerSec * 0.5 * config.wheelDiameter.in(Meter),
            new Rotation2d(data.steerMotorPositionRad));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            data.driveMotorPositionRad * config.wheelDiameter.in(Meter) * 0.5,
            new Rotation2d(data.steerMotorPositionRad));
    }

    public SwerveModulePosition[] getSampledSwerveModulePositions() {
        return odometryPositions;
    }
}