package org.fairportrobotics.frc.robolib.drivesystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private final int CAN_UPDATE_FREQUENCY = 150;
    private final double STEER_MAX_CURRENT = 60;
    private final double DRIVE_MAX_CURRENT = 80;

    private final double gearRatio;
    private final double wheelDiameterInMeters;

    private SwerveModuleState requestedModuleState;

    private TalonFX driveMotor;
    private VelocityVoltage driveRequest = new VelocityVoltage(0);

    private TalonFX steerMotor;
    private PositionVoltage steerPosRequest = new PositionVoltage(0);

    private CANcoder steerEncoder;

    private String moduleName;

    private Translation2d moduleLocation;

    public SwerveModule(
        int driveMotorId,
        double driveKP,
        double driveKI,
        double driveKD,
        double driveKV,
        boolean driveInverted,
        int steerMotorId,
        double steerKP,
        double steerKI,
        double steerKD,
        double steerKS,
        double steerKV,
        double steerKA,
        int steerEncoderId,
        double steerOffset,
        CANBus canBusObj,
        Translation2d moduleLocation,
        double gearRatio,
        double wheelDiameterInMeters,
        String moduleName
    ){

        steerEncoder = new CANcoder(steerEncoderId, canBusObj);
        steerEncoder.getAbsolutePosition().setUpdateFrequency(CAN_UPDATE_FREQUENCY);
        steerEncoder.optimizeBusUtilization();
        steerEncoder.getConfigurator().apply(generateCanCoderConfiguration(steerOffset));

        driveMotor = new TalonFX(driveMotorId, canBusObj);
        driveMotor.getVelocity().setUpdateFrequency(CAN_UPDATE_FREQUENCY);
        driveMotor.optimizeBusUtilization();
        driveMotor.getConfigurator().apply(generateDriveTalonConfiguration(driveKP, driveKI, driveKD, driveKV, driveInverted));

        steerMotor = new TalonFX(steerMotorId, canBusObj);
        steerMotor.optimizeBusUtilization();
        steerMotor.getConfigurator().apply(generateSteerTalonConfiguration(steerKP, steerKI, steerKD, steerKS, steerKV, steerKA));
        // steerMotor.setNeutralMode(NeutralModeValue.Brake);

        this.moduleLocation = moduleLocation;

        this.gearRatio = gearRatio;
        this.wheelDiameterInMeters = wheelDiameterInMeters;

        this.moduleName = moduleName;
    }

    public void setSteerRotations(Rotation2d rotations){
        steerMotor.setControl(steerPosRequest.withPosition(rotations.getRotations()));
    }

    public Rotation2d getSteerRotations(){
        return new Rotation2d(steerEncoder.getAbsolutePosition().getValue());
    }

    public void setDriveSpeed(double speed){
        // Convert wheel speed to rotor speed
        // (speed / Circumfrence of wheel) * gear ratio
        double rotorVelocity = (speed / (Math.PI * wheelDiameterInMeters)) * gearRatio;
        driveMotor.setControl(driveRequest.withSlot(0).withVelocity(rotorVelocity));
    }

    /**
     * Get drive motor speed in rotations per second
     * @return
     */
    public double getDriveMotorSpeed(){
        return driveMotor.getVelocity().refresh().getValueAsDouble();
    }

    /**
     * Get drive wheel speed in meters per second
     * @return
     */
    public double getDriveWheelSpeed(){
        return (driveMotor.getVelocity().refresh().getValueAsDouble() / gearRatio) * (Math.PI * wheelDiameterInMeters);
    }

    public Translation2d getModuleLocation(){
        return moduleLocation;
    }

    public void setRequestedModuleState(SwerveModuleState state){
        this.setSteerRotations(state.angle);
        this.setDriveSpeed(state.speedMetersPerSecond);
        requestedModuleState = state;
    }

    public TalonFX getSteerMotor(){
        return steerMotor;
    }

    public TalonFX getDriveMotor(){
        return driveMotor;
    }

    public CANcoder getSteerEncoder(){
        return steerEncoder;
    }

    public SwerveModuleState getRequestedModuleState(){
        return requestedModuleState;
    }

    public SwerveModuleState getActualModuleState() {
        return new SwerveModuleState(getDriveWheelSpeed(), getSteerRotations());
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getModuleDistance(), getSteerRotations());
    }

    public void periodic(){
        SmartDashboard.putNumber(moduleName + " steerPow", steerMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber(moduleName + " steer error", steerMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber(moduleName + " steer setpoint", steerMotor.getClosedLoopReference().getValueAsDouble());

        SmartDashboard.putNumber(moduleName + " steerPos", getSteerRotations().getRotations());
    }

    public String getModuleName(){
        return moduleName;
    }

    private double getModuleDistance(){
        double distance = (this.driveMotor.getPosition(true).getValueAsDouble() / gearRatio) * (Math.PI * wheelDiameterInMeters);
        return distance;
    }

    private TalonFXConfiguration generateDriveTalonConfiguration(double kP, double kI, double kD, double kV, boolean driveInverted){
        return new TalonFXConfiguration().withSlot0(
            new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKV(kV)
            )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(DRIVE_MAX_CURRENT)
        );
    }

    private TalonFXConfiguration generateSteerTalonConfiguration(double kP, double kI, double kD, double kS, double kV, double kA){
        return new TalonFXConfiguration().withSlot0(
            new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(160)
                .withMotionMagicAcceleration(160)
                .withMotionMagicJerk(1600)
        ).withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(steerEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(1.0)
                .withRotorToSensorRatio(12.8)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
            .withContinuousWrap(true)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(STEER_MAX_CURRENT)
        );
    }

    private CANcoderConfiguration generateCanCoderConfiguration(double steerOffset){
        return new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs()
                .withMagnetOffset(-steerOffset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
            );
    }

}
