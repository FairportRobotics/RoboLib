package org.fairportrobotics.frc.robolib.drivesystems.swerve;

import java.util.Arrays;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase{

    private SwerveDriveKinematics driveKiniematics;
    private ChassisSpeeds chassisSpeeds;
    private Translation2d centerOfRotation;

    private SwerveDrivePoseEstimator3d poseEstimator;

    private SwerveModule[] modules;

    private double MAX_LINEAR_SPEED = 3.9;
    private double MAX_ANGULAR_SPEED = Math.PI * 2;

    private Pigeon2 gyro;

    private double mLastSimTime = 0;

    public SwerveDriveSubsystem(int pigeonId, CANBus canBus, double maxLinearVelMetersSecond, double maxAngularVelRadiansSecond, SwerveModule... modules){

        MAX_LINEAR_SPEED = maxLinearVelMetersSecond;
        MAX_ANGULAR_SPEED = maxAngularVelRadiansSecond;

        this.modules = modules; 

        driveKiniematics = new SwerveDriveKinematics(
            Arrays.stream(modules).map((m) -> m.getModuleLocation()).toArray(size -> new Translation2d[size])
        );

        gyro = new Pigeon2(pigeonId, canBus);

        poseEstimator = new SwerveDrivePoseEstimator3d(driveKiniematics, new Rotation3d(Rotation2d.fromRotations(getCurrentYaw().magnitude())), getModulePositions(), Pose3d.kZero);
        chassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveModuleState[] moduleStates = driveKiniematics.toSwerveModuleStates(chassisSpeeds);

        for(int i=0; i<modules.length;i++){
            moduleStates[i].optimize(modules[i].getSteerRotations());
        }

        poseEstimator.updateWithTime(Utils.getCurrentTimeSeconds(), new Rotation3d(Rotation2d.fromRotations(getCurrentYaw().magnitude())), getModulePositions());

        for(int i=0;i<modules.length;i++){
            modules[i].setRequestedModuleState(moduleStates[i]);
            modules[i].periodic();
        }
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        double deltaTimeSecond = (mLastSimTime - Utils.getCurrentTimeSeconds());

        if(mLastSimTime != 0){
            Pose3d oldPose = getRobotPose();
            poseEstimator.resetPose(new Pose3d(oldPose.getX() + (chassisSpeeds.vxMetersPerSecond * (deltaTimeSecond)), oldPose.getY() + chassisSpeeds.vyMetersPerSecond * (deltaTimeSecond), 0, oldPose.getRotation().plus(new Rotation3d(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * (deltaTimeSecond))))));
        }

        mLastSimTime = Utils.getCurrentTimeSeconds();
    }

    public Translation2d[] getModuleLocations(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getModuleLocation()).toArray(Translation2d[]::new);
    }

    public SwerveModuleState[] getRequestedModuleStates(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getRequestedModuleState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModuleState[] getActualModuleStates(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getActualModuleState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModule[] getModules(){
        return Arrays.stream(modules).toArray(SwerveModule[]::new);
    }

    public int getNumberOfModules(){
        return modules.length;
    }

    public Angle getCurrentYaw(){
        return gyro.getRoll().getValue();
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
        this.setChassisSpeed(chassisSpeeds, new Translation2d());
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        this.chassisSpeeds = chassisSpeeds;
        this.centerOfRotation = centerOfRotation;
    }

    /**
     * Set robot speed relative to field
     * @param x Forward robot speed in meters per second
     * @param y Left robot speed in meters per second
     * @param rot CCW robot speed in radians per second
     */
    public void setChassisSpeedsFromJoystickFieldRelative(double x, double y, double rot){
        this.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(
            MAX_LINEAR_SPEED * x,
            MAX_LINEAR_SPEED * y,
            MAX_ANGULAR_SPEED * rot,
            Rotation2d.fromDegrees(getCurrentYaw().magnitude())
        ));
    }

    /**
     * Set robot speed relative to robot
     * @param x Forward robot speed in meters per second
     * @param y Left robot speed in meters per second
     * @param rot CCW robot speed in radians per second
     */
    public void setChassisSpeedsFromJoystickRobotRelative(double x, double y, double rot){
        this.setChassisSpeed(ChassisSpeeds.fromRobotRelativeSpeeds(
            MAX_LINEAR_SPEED * x,
            MAX_LINEAR_SPEED * y,
            MAX_ANGULAR_SPEED * rot,
            Rotation2d.fromDegrees(getCurrentYaw().magnitude())
        ));
    }

    public Pose3d getRobotPose(){
        return poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getModulePositions(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

}
