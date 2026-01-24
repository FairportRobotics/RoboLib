package org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive;

import java.util.Arrays;

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

    private final double MAX_LINEAR_SPEED = 3.9;
    private final double MAX_ANGULAR_SPEED = Math.PI * 2;

    private Pigeon2 gyro;

    public SwerveDriveSubsystem(int pigeonId, SwerveModule... modules){

        this.modules = modules; 

        driveKiniematics = new SwerveDriveKinematics(
            Arrays.stream(modules).map((m) -> m.getModuleLocation()).toArray(size -> new Translation2d[size])
        );

        gyro = new Pigeon2(pigeonId);

        poseEstimator = new SwerveDrivePoseEstimator3d(driveKiniematics, new Rotation3d(Rotation2d.fromRotations(getCurrentYaw().magnitude())), getModulePositions(), Pose3d.kZero);
        chassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public void periodic() {
        SwerveModuleState[] moduleStates = driveKiniematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

        for(int i=0; i<modules.length;i++){
            moduleStates[i].optimize(modules[i].getCurrentSteerRotations());
        }

        poseEstimator.update(new Rotation3d(Rotation2d.fromRotations(getCurrentYaw().magnitude())), getModulePositions());

        for(int i=0;i<modules.length;i++){
            modules[i].setModuleState(moduleStates[i]);
        }
    }

    public Translation2d[] getModuleLocations(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getModuleLocation()).toArray(Translation2d[]::new);
    }

    public SwerveModuleState[] getModuleStates(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getModuleState()).toArray(SwerveModuleState[]::new);
    }

    public Angle getCurrentYaw(){
        return gyro.getYaw().refresh().getValue();
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
        this.setChassisSpeed(chassisSpeeds, Translation2d.kZero);
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        this.chassisSpeeds = chassisSpeeds;
        this.centerOfRotation = centerOfRotation;
    }

    public void setChassisSpeedsFromJoystick(double x, double y, double rot){
        this.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(
            MAX_LINEAR_SPEED * x,
            MAX_LINEAR_SPEED * y,
            MAX_ANGULAR_SPEED * rot,
            Rotation2d.fromDegrees(getCurrentYaw().magnitude())
        ));
    }

    private SwerveModulePosition[] getModulePositions(){
        return Arrays.stream(modules).map((SwerveModule m) -> m.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

}
