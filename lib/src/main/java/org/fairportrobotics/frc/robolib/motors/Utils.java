package org.fairportrobotics.frc.robolib.motors;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * Class defining static helper methods for motors
 */
public class Utils {
    public static void SetMotorDirection(TalonFX motor, InvertedValue direction) {
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.refresh(outputConfigs);
        outputConfigs.Inverted = direction;
        configurator.apply(outputConfigs);
    }
}
