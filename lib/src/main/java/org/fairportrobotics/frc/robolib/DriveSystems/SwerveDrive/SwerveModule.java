package org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.sun.jdi.request.StepRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final int CAN_UPDATE_FREQUENCY = 50;

    private final int TALON_BUILT_IN_ENCODER_UNITS_PER_ROTATION = 2048;
    private final int CANCODER_UNITS_PER_ROTATION = 2048;

    private final double gearRatio;
    private final double wheelDiameterInMeters;

    private SwerveModuleState moduleState;

    private TalonFX driveMotor;
    private VelocityVoltage driveRequest = new VelocityVoltage(0);
    private TalonFX steerMotor;
    private PositionVoltage steerRequest = new PositionVoltage(0);
    private CANcoder steerEncoder;

    private String moduleName;

    private Translation2d moduleLocation;

    public SwerveModule(
        int driveMotorId,
        double driveKP,
        double driveKI,
        double driveKD,
        int steerMotorId,
        double steerKP,
        double steerKI,
        double steerKD,
        int steerEncoderId,
        double steerOffset,
        String canBusName,
        Translation2d moduleLocation,
        double gearRatio,
        double wheelDiameterInMeters,
        String moduleName
    ){
        driveMotor = new TalonFX(driveMotorId, canBusName);
        driveMotor.getVelocity().setUpdateFrequency(CAN_UPDATE_FREQUENCY);
        driveMotor.optimizeBusUtilization();
        driveMotor.getConfigurator().apply(generateTalonConfiguration(driveKP, driveKI, driveKD));

        steerMotor = new TalonFX(steerMotorId, canBusName);
        steerMotor.optimizeBusUtilization();
        steerMotor.getConfigurator().apply(generateTalonConfiguration(steerKP, steerKI, steerKD));

        steerEncoder = new CANcoder(steerEncoderId, canBusName);
        steerEncoder.getPosition().setUpdateFrequency(CAN_UPDATE_FREQUENCY);
        steerEncoder.optimizeBusUtilization();
        steerEncoder.getConfigurator().apply(generateCanCoderConfiguration(steerOffset));

        this.moduleLocation = moduleLocation;

        this.gearRatio = gearRatio;
        this.wheelDiameterInMeters = wheelDiameterInMeters;

        this.moduleName = moduleName;
    }

    public void setSteerRotations(double rotations){
        steerMotor.setControl(steerRequest.withSlot(0).withPosition(rotations));
    }

    public Rotation2d getCurrentSteerRotations(){
        return new Rotation2d(steerEncoder.getPosition().refresh().getValueAsDouble());
    }

    public void setDriveSpeed(double speed){
        double rotorVelocity = ((Math.PI * wheelDiameterInMeters) / speed) * gearRatio;
        //driveMotor.setControl(driveRequest.withSlot(0).withVelocity(rotorVelocity));
    }

    public double getDriveSpeed(){
        return driveMotor.getVelocity().refresh().getValueAsDouble();
    }

    public Translation2d getModuleLocation(){
        return moduleLocation;
    }

    public void setModuleState(SwerveModuleState state){
        this.setSteerRotations(state.angle.getRotations());
        this.setDriveSpeed(state.speedMetersPerSecond);
        moduleState = state;
    }

    public SwerveModuleState getModuleState(){
        return moduleState;
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getModuleDistance(), getCurrentSteerRotations());
    }

    private double getModuleDistance(){
        double distance = (this.driveMotor.getPosition(true).getValueAsDouble()) / gearRatio / (Math.PI * wheelDiameterInMeters);
        return distance;
    }

    private TalonFXConfiguration generateTalonConfiguration(double kP, double kI, double kD){
        return new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
    }

    private CANcoderConfiguration generateCanCoderConfiguration(double steerOffset){
        return new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(steerOffset));
    }

}
