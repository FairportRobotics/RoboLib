package org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase{

    private SwerveDriveKinematics driveKiniematics;
    private ChassisSpeeds chassisSpeeds;
    private Translation2d centerOfRotation;

    private SwerveModule[] modules;

    private final double MAX_LINEAR_SPEED = 3.9;
    private final double MAX_ANGULAR_SPEED = Math.PI * 2;

    private Pigeon2 gyro;

    public SwerveDriveSubsystem(){

        driveKiniematics = new SwerveDriveKinematics(
            modules[0].getModuleLocation(),
            modules[1].getModuleLocation(),
            modules[2].getModuleLocation(),
            modules[3].getModuleLocation()
        );

    }

    @Override
    public void periodic() {
        SwerveModuleState[] moduleStates = driveKiniematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        Logger.recordOutput("Un-Optimized Swerve States", moduleStates);

        for(int i=0; i<modules.length;i++){
            moduleStates[i].optimize(Rotation2d.fromRotations(modules[i].getCurrentSteerRotations()));
        }

        Logger.recordOutput("Optimized Swerve States", moduleStates);

        for(int i=0;i<modules.length;i++){
            modules[i].setSteerRotations(moduleStates[i].angle.getRotations());
            modules[i].setDriveSpeed(moduleStates[i].speedMetersPerSecond);
        }
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
        this.chassisSpeeds = chassisSpeeds;
    }

    public void setChassisSpeed(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        this.chassisSpeeds = chassisSpeeds;
        this.centerOfRotation = centerOfRotation;
    }

    public void chassisSpeedsFromJoystick(double x, double y, double rot){
        ChassisSpeeds.fromFieldRelativeSpeeds(MAX_LINEAR_SPEED * x, MAX_LINEAR_SPEED * y, MAX_ANGULAR_SPEED * rot, Rotation2d.fromDegrees(gyro.getYaw().refresh().getValueAsDouble()));
    }

}
