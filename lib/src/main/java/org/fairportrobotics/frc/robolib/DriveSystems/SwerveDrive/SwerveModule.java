package org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.sun.jdi.request.StepRequest;

import edu.wpi.first.math.geometry.Translation2d;
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
        String canBusName,
        Translation2d moduleLocation,
        double gearRatio,
        double wheelDiameterInMeters
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
        steerEncoder.getConfigurator().apply(generateCanCoderConfiguration());

        this.moduleLocation = moduleLocation;

        this.gearRatio = gearRatio;
        this.wheelDiameterInMeters = wheelDiameterInMeters;
    }

    public void setSteerRotations(double rotations){
        steerMotor.setControl(steerRequest.withSlot(0).withPosition(rotations));
    }

    public double getCurrentSteerRotations(){
        return steerEncoder.getPosition().refresh().getValueAsDouble();
    }

    public void setDriveSpeed(double speed){
        double rotorVelocity = ((Math.PI * wheelDiameterInMeters) / speed) * gearRatio * TALON_BUILT_IN_ENCODER_UNITS_PER_ROTATION;
        driveMotor.setControl(driveRequest.withSlot(0).withVelocity(rotorVelocity));
    }

    public double getDriveSpeed(){
        return driveMotor.getVelocity().refresh().getValueAsDouble();
    }

    public Translation2d getModuleLocation(){
        return moduleLocation;
    }

    public SwerveModuleState getModuleState(){
        return moduleState;
    }

    private TalonFXConfiguration generateTalonConfiguration(double kP, double kI, double kD){
        return new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
    }

    private CANcoderConfiguration generateCanCoderConfiguration(){
        return new CANcoderConfiguration();
    }

}
