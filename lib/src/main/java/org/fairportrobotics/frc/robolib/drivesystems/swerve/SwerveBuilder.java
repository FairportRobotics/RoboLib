package org.fairportrobotics.frc.robolib.drivesystems.swerve;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveBuilder {

    private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();
    private CANBus canBusObj;

    private double maxAngularVelRadiansSecond = 1.0;
    private double maxLinearVelMetersSecond = Math.PI;

    private int pigeonId;

    public SwerveBuilder() {

    }

    /**
     * Set the CAN ID of the Pigeon IMU
     * @param pigeonId CAN Bus ID
     * @return itself
     */
    public SwerveBuilder withPigeonId(int pigeonId) {
        this.pigeonId = pigeonId;
        return this;
    }

    /**
     * Set the CAN Bus name
     * @param canBusname Name of the CAN Bus
     * @return itself
     */
    public SwerveBuilder withCanbusName(String canBusname) {
        this.canBusObj = new CANBus(canBusname);
        return this;
    }

    /**
     * Add a SwerveModule
     * 
     * Use the inner class SwerveModuleBuilder to construct a SwerveModule
     * @param module 
     * @return itself
     */
    public SwerveBuilder withSwerveModule(SwerveModule module) {
        this.modules.add(module);
        return this;
    }

    /**
     * Set the max linear velocity in meters per second
     * 
     * @param maxVel Max linear velocity in meters per second
     * @return itself
     */
    public SwerveBuilder withMaxLinearVelocity(double maxVel) {
        this.maxLinearVelMetersSecond = maxVel;
        return this;
    }

    /**
     * Set the max angular velocity in radians per second
     * 
     * @param maxVel Max angular velocity in radians per second
     * @return itself
     */
    public SwerveBuilder withMaxAngularVelocity(double maxVel) {
        this.maxAngularVelRadiansSecond = maxVel;
        return this;
    }

    /**
     * Build the configured instance of a SwerveDriveSystem
     * @return instance of SwerveDriveSystem
     */
    public SwerveDriveSystem build() {
        return new SwerveDriveSystem(pigeonId, canBusObj, maxLinearVelMetersSecond, maxAngularVelRadiansSecond,
                modules.toArray(size -> new SwerveModule[size]));
    }

    public class SwerveModuleBuilder {

        private int driveMotorId;
        private double driveKP;
        private double driveKI;
        private double driveKD;
        private double driveKV;
        private boolean driveInverted = false;
        private int steerMotorId;
        private double steerKP;
        private double steerKI;
        private double steerKD;
        private double steerKS;
        private double steerKV;
        private double steerKA;
        private int steerEncoderId;
        private double steerOffset;
        private Translation2d moduleLocation;
        private double gearRatio;
        private double wheelDiameterInMeters;
        private String moduleName;

        public SwerveModuleBuilder() {

        }

        /**
         * Set the drive motor CAN Bus ID
         * @param driveMotorId ID of the drive motor
         * @return itself
         */
        public SwerveModuleBuilder withDriveMotorId(int driveMotorId) {
            this.driveMotorId = driveMotorId;
            return this;
        }

        /**
         * Set the drive motors closed loop proportional gain value.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * 
         * @param driveKP Proportional gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withDriveKP(double driveKP) {
            this.driveKP = driveKP;
            return this;
        }


        /**
         * Set the drive motors closed loop integral gain value.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * 
         * @param driveKI Integral gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withDriveKI(double driveKI) {
            this.driveKI = driveKI;
            return this;
        }


        /**
         * Set the drive motors closed loop derrivative gain value.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * 
         * @param driveKD Derrivative gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withDriveKD(double driveKD) {
            this.driveKD = driveKD;
            return this;
        }

        /**
         * Set the drive motors closed loop feed forward velocity gain.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param driveKV velocity gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withDriveKV(double driveKV) {
            this.driveKV = driveKV;
            return this;
        }

        /**
         * Set the drive motors direction to be inverted.
         * <br><br>
         * When inverted positive values will be Counter Clockwise
         * 
         * @return itself
         */
        public SwerveModuleBuilder withDriveInverted(){
            this.driveInverted = true;
            return this;
        }

        /**
         * Set the steer motors CAN Bus ID
         * @param steerMotorId ID of steer motor
         * @return itself
         */
        public SwerveModuleBuilder withSteerMotorId(int steerMotorId) {
            this.steerMotorId = steerMotorId;
            return this;
        }

        /**
         * Set the steer motors closed loop proportional gain value
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param steerKP Proportional gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withSteerKP(double steerKP) {
            this.steerKP = steerKP;
            return this;
        }

        /**
         * Set the steer motors closed loop integral gain value
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param steerKI Integral gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withSteerKI(double steerKI) {
            this.steerKI = steerKI;
            return this;
        }

        /**
         * Set the steer motors closed loop derrivative gain value
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param steerKD Derrivative gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withSteerKD(double steerKD) {
            this.steerKD = steerKD;
            return this;
        }

        /**
         * Set the steer motors closed loop feed forward static gain.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param steerKS Static gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withSteerKS(double steerKS) {
            this.steerKS = steerKS;
            return this;
        }

        /**
         * Set the steer motors closed loop feed forward velocity gain.
         * <br><br>
         * Refer here for more info: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html
         * @param steerKV velocity gain in Volts
         * @return itself
         */
        public SwerveModuleBuilder withSteerKV(double steerKV) {
            this.steerKV = steerKV;
            return this;
        }

        /**
         * Set the steer encoders CAN Bus ID
         * @param steerEncoderId ID of steer encoder
         * @return itself
         */
        public SwerveModuleBuilder withSteerEncoderId(int steerEncoderId) {
            this.steerEncoderId = steerEncoderId;
            return this;
        }

        /**
         * Set the offset of the steer encoder.
         * <br><br>
         * This value can be found from Pheonix tuner.
         * <br><br>
         * Ensure the configured offset is 0.0 and then set this to
         * be the Absolute Position when the wheel is properly aligned
         * @param offsetRotations offset in the range of [-0.5, 0.5)
         * @return itself
         */
        public SwerveModuleBuilder withSteerOffset(double offsetRotations) {
            this.steerOffset = offsetRotations;
            return this;
        }

        /**
         * Set the swerve modules location relative to the center of the robot.
         * <br><br>
         * Refer to https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
         * for how the cordinate system works.
         * <br><br>
         * The location is in meters from the center of the robot.
         * 
         * @param moduleLocation location of the swerve module
         * @return
         */
        public SwerveModuleBuilder withModuleLocation(Translation2d moduleLocation) {
            this.moduleLocation = moduleLocation;
            return this;
        }

        /**
         * Set the gear ratio of the drive motor to the drive wheel.
         * <br><br>
         * This ratio is the number of rotations the motor has to make for one(1) rotation
         * of the drive wheel.
         * @param gearRatio the gear ratio
         * @return itself
         */
        public SwerveModuleBuilder withGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        /**
         * Set the diameter of the drive wheel in meters.
         * @param wheelDiameterInMeters diameter of the drive wheel
         * @return itself
         */
        public SwerveModuleBuilder withWheelDiameter(double wheelDiameterInMeters) {
            this.wheelDiameterInMeters = wheelDiameterInMeters;
            return this;
        }

        /**
         * Set the friendly name of this module. This name is used in logging
         * @param moduleName name of the module
         * @return itself
         */
        public SwerveModuleBuilder withModuleName(String moduleName) {
            this.moduleName = moduleName;
            return this;
        }

        /**
         * Build the configured instance of a SwerveModule
         * @return a SwerveModule object
         */
        public SwerveModule build() {
            return new SwerveModule(driveMotorId, driveKP, driveKI, driveKD, driveKV, driveInverted, steerMotorId, steerKP, steerKI,
                    steerKD, steerKS, steerKV, steerKA,
                    steerEncoderId, steerOffset, canBusObj, moduleLocation, gearRatio, wheelDiameterInMeters,
                    moduleName);
        }

    }

}
