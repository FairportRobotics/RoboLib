package org.fairportrobotics.frc.robolib.controllers.lighting;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.SerialPort;


public class ArduinoLightingController {
    
    final SerialPort bus;
    final Pattern colorPattern = Pattern.compile("\\d{9}");
    private final String NULL_COLOR = "000000000";
    private final String RAINBOW_VALUE =  NULL_COLOR + "254";
    private final String SHIFT_VALUE = NULL_COLOR + "251";
    private final String SHIFT_WRAP_VALUE = NULL_COLOR + "250";

    public ArduinoLightingController(int baudRate, SerialPort.Port port) {
        this.bus = new SerialPort(baudRate, port);
    }

    private boolean validateColor(String colorValue) {
        Matcher m = colorPattern.matcher(colorValue);
        return m.find();
    }
    public int executeColor(String colorValue, String command) {
        if ( validateColor(colorValue) ) {
            return bus.writeString( colorValue + command );
        }
        return 0;
    }

    public int fillAll(String colorValue) {
        return executeColor(colorValue, "255");
    }

    public int fillRainbow() {
        return bus.writeString(RAINBOW_VALUE);
    }

    public int switchGradient(String colorValue) {
        return executeColor(colorValue, "253");
    }

    public int fillShift(String colorValue) {
        return executeColor(colorValue, "252");
    }

    public int shift() {
        return bus.writeString(SHIFT_VALUE);
    }

    public int shiftWrap() {
        return bus.writeString(SHIFT_WRAP_VALUE);
    }

    public int flash(String colorValue) {
        return executeColor(colorValue, "249");
    }


}
