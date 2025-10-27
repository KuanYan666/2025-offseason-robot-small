package lib.ironpulse.swerve;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import java.util.*;
import java.util.concurrent.locks.*;
import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final SwerveConfig config;
    private final List<SwerveModule> modules;
    private final ImuIO imuIO;
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator setpointGenerator;
    private final SwerveDrivePoseEstimator3d poseEstimator;
    private final List<Rotation2d> xLockAngles;
    private final ImuIOInputsAutoLogged imuInputs;
    private SwerveSetpoint setpointCurr;
    @Getter private Voltage previouslyAppliedVoltage;
    private MODE mode = MODE.VELOCITY;

    public Swerve(SwerveConfig config, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        this.config = config;
        this.imuIO = imuIO;
        this.imuInputs = new ImuIOInputsAutoLogged();
        
        // Create modules
        this.modules = new ArrayList<>();
        for (int i = 0; i < config.moduleConfigs.length; i++)
            modules.add(new SwerveModule(config, config.moduleConfigs[i], moduleIOs[i]));
        
        // Setup kinematics
        kinematics = new SwerveDriveKinematics(config.moduleLocations());
        setpointGenerator = SwerveSetpointGenerator.builder()
            .kinematics(kinematics)
            .chassisLimit(config.defaultSwerveLimit)
            .moduleLimit(config.defaultSwerveModuleLimit)
            .build();
        
        // Initial setpoint
        var states = new SwerveModuleState[modules.size()];
        for (int i = 0; i < states.length; i++)
            states[i] = modules.get(i).getSwerveModuleState();
        setpointCurr = new SwerveSetpoint(new ChassisSpeeds(), states);
        
        // Pose estimator
        poseEstimator = new SwerveDrivePoseEstimator3d(
            kinematics, new Rotation3d(), getModulePositions(), new Pose3d());
        
        // X-lock angles
        xLockAngles = new ArrayList<>();
        for (var loc : config.moduleLocations())
            xLockAngles.add(loc.getAngle());
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        imuInputs.yawVelocityRadPerSecCmd = getChassisSpeeds().omegaRadiansPerSecond;
        imuIO.updateInputs(imuInputs);
        Logger.processInputs(config.name + "/IMU", imuInputs);
        modules.forEach(m -> { m.updateInputs(); m.periodic(); });
        
        // Update pose estimator
        var samples = getSampledModulePositions();
        var rotations = imuInputs.odometryRotations;
        for (int i = 0; i < samples.size(); i++)
            poseEstimator.updateWithTime(Timer.getTimestamp(), rotations[i], samples.get(i).getSecond());
        odometryLock.unlock();
        
        // Logging (minimal)
        Logger.recordOutput(config.name + "/Pose", poseEstimator.getEstimatedPosition());
    }

    public void runTwist(ChassisSpeeds speeds) {
        mode = MODE.VELOCITY;
        setpointCurr = setpointGenerator.generate(speeds, setpointCurr, config.dtS);
        for (int i = 0; i < modules.size(); i++)
            modules.get(i).runState(setpointCurr.moduleStates()[i]);
    }

    public void runTwistWithTorque(ChassisSpeeds speeds, Current[] tau) {
        mode = MODE.VELOCITY;
        setpointCurr = setpointGenerator.generate(speeds, setpointCurr, config.dtS);
        for (int i = 0; i < modules.size(); i++)
            modules.get(i).runState(setpointCurr.moduleStates()[i], tau[i]);
    }

    public void runVoltage(Voltage voltage) {
        mode = MODE.VOLTAGE;
        previouslyAppliedVoltage = voltage;
        modules.forEach(m -> m.runDriveVoltage(voltage));
    }

    public void runStop() { runVoltage(Volt.of(0.0)); }

    public void runStopAndLock() {
        kinematics.resetHeadings(xLockAngles.toArray(new Rotation2d[0]));
        runStop();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
            modules.stream().map(SwerveModule::getSwerveModuleState).toArray(SwerveModuleState[]::new));
    }

    private SwerveModuleState[] getModuleStates() {
        return modules.stream().map(SwerveModule::getSwerveModuleState).toArray(SwerveModuleState[]::new);
    }

    private SwerveModulePosition[] getModulePositions() {
        return modules.stream().map(SwerveModule::getSwerveModulePosition).toArray(SwerveModulePosition[]::new);
    }

    public List<Pair<Double, SwerveModulePosition[]>> getSampledModulePositions() {
        var timestamps = imuInputs.odometryYawTimestamps;
        var samplesByModule = modules.stream().map(SwerveModule::getSampledSwerveModulePositions).toList();
        var result = new ArrayList<Pair<Double, SwerveModulePosition[]>>();
        for (int i = 0; i < timestamps.length; i++) {
            var positions = new SwerveModulePosition[modules.size()];
            for (int j = 0; j < modules.size(); j++)
                positions[j] = samplesByModule.get(j)[i];
            result.add(new Pair<>(timestamps[i], positions));
        }
        return result;
    }

    public Pose3d getEstimatedPose() {
       return poseEstimator.getEstimatedPosition(); 
      }

    public void resetEstimatedPose(Pose3d pose) {
       poseEstimator.resetPose(pose); 
      }

    public Optional<Pose3d> getEstimatedPoseAt(Time time) {
       return poseEstimator.sampleAt(time.in(Seconds)); 
      }
    
    public void addVisionMeasurement(Pose3d pose, double time, Matrix<N4, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(pose, time, stdDevs);
    }

    public SwerveLimit getSwerveLimit() {
       return setpointGenerator.getChassisLimit(); 
      }

    public void setSwerveLimit(SwerveLimit limit) {
       setpointGenerator.setChassisLimit(limit);
      }

    public void setSwerveLimitDefault() {
       setpointGenerator.setChassisLimit(config.defaultSwerveLimit); 
      }

    public SwerveModuleLimit getSwerveModuleLimit() {
       return setpointGenerator.getModuleLimit(); 
      }

    public void setSwerveModuleLimit(SwerveModuleLimit limit) {
       setpointGenerator.setModuleLimit(limit); 
      }

    public void setSwerveModuleLimitDefault() {
       setpointGenerator.setModuleLimit(config.defaultSwerveModuleLimit); 
      }

    public enum MODE { VELOCITY, VOLTAGE }
}