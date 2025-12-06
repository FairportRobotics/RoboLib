package org.fairportrobotics.frc.robolib.DriveSystems.SwerveDrive;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveBuilder {

    ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();
    String canBusName = "";

    private int pigeonId;

    public SwerveBuilder(){

    }

    public SwerveBuilder withPigeonId(int pigeonId){
        this.pigeonId = pigeonId;
        return this;
    }

    public SwerveBuilder withCanbusName(String canBusname){
        this.canBusName = canBusname;
        return this;
    }

    public SwerveBuilder withSwerveModule(SwerveModule module){
        this.modules.add(module);
        return this;
    }

    public SwerveDriveSubsystem build(){
        return new SwerveDriveSubsystem(pigeonId, modules.toArray(size -> new SwerveModule[size]));
    }

    public class SwerveModuleBuilder {

        private int driveMotorId;
        private double driveKP;
        private double driveKI;
        private double driveKD;
        private int steerMotorId;
        private double steerKP;
        private double steerKI;
        private double steerKD;
        private int steerEncoderId;
        private double steerOffset;
        private Translation2d moduleLocation;
        private double gearRatio;
        private double wheelDiameterInMeters;

        public SwerveModuleBuilder() {

        }

        public SwerveModuleBuilder withDriveMotorId(int driveMotorId){
            this.driveMotorId = driveMotorId;
            return this;
        }

        public SwerveModuleBuilder withDriveKP(double driveKP){
            this.driveKP = driveKP;
            return this;
        }

        public SwerveModuleBuilder withDriveKI(double driveKI){
            this.driveKI = driveKI;
            return this;
        }

        public SwerveModuleBuilder withDriveKD(double driveKD){
            this.driveKD = driveKD;
            return this;
        }

        public SwerveModuleBuilder withSteerMotorId(int steerMotorId){
            this.steerMotorId = steerMotorId;
            return this;
        }

        public SwerveModuleBuilder withSteerKP(double steerKP){
            this.steerKP = steerKP;
            return this;
        }

        public SwerveModuleBuilder withSteerKI(double steerKI){
            this.steerKI = steerKI;
            return this;
        }

        public SwerveModuleBuilder withSteerKD(double steerKD){
            this.steerKD = steerKD;
            return this;
        }

        public SwerveModuleBuilder withSteerEncoderId(int steerEncoderId){
            this.steerEncoderId = steerEncoderId;
            return this;
        }

        public SwerveModuleBuilder withModuleLocation(Translation2d moduleLocation){
            this.moduleLocation = moduleLocation;
            return this;
        }

        public SwerveModuleBuilder withGearRatio(double gearRatio){
            this.gearRatio = gearRatio;
            return this;
        }

        public SwerveModuleBuilder withWheelDiameter(double wheelDiameterInMeters){
            this.wheelDiameterInMeters = wheelDiameterInMeters;
            return this;
        }

        public SwerveModule build() {
            return new SwerveModule(driveMotorId, driveKP, driveKI, driveKD, steerMotorId, steerKP, steerKI, steerKD,
                    steerEncoderId, steerOffset, canBusName, moduleLocation, gearRatio, wheelDiameterInMeters);
        }

    }

}
