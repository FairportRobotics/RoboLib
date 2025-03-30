package org.fairportrobotics.frc.robolib.sensors.colorsensors;

import edu.wpi.first.wpilibj.I2C;

/**
 * 
 */
public class TCS34725 {

    /** */
    public class TCS34725_RGB {
        private int r, g, b;

        public TCS34725_RGB(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public int getR() {
            return this.r;
        }

        public int getG() {
            return this.g;
        }

        public int getB() {
            return this.b;
        }
    }

    /**
     * An Enum reprenting the GAIN values for the TCS34725.
     */
    public enum TCS34721_GAIN {
        /** No gain */
        TCS34725_GAIN_1X(0x00),
        /** 4x gain */
        TCS34725_GAIN_4X(0x01),
        /** 16x gain */
        TCS34725_GAIN_16X(0x02),
        /** 60x gain */
        TCS34725_GAIN_60X(0x03);

        public final int value;

        private TCS34721_GAIN(int i) {
            this.value = i;
        };
    };


    /**   I2C address **/
    public static int TCS34725_ADDRESS = (0x29);
    /**   Command bit **/
    public static int TCS34725_COMMAND_BIT = (0x80);
    /**   Interrupt Enable register */
    public static int TCS34725_ENABLE = (0x00);
    /**   RGBC Interrupt Enable */
    public static int TCS34725_ENABLE_AIEN = (0x10);
    /**   Wait Enable - Writing 1 activates the wait timer */
    public static int TCS34725_ENABLE_WEN = (0x08);
    /**   RGBC Enable - Writing 1 actives the ADC, 0 disables it */
    public static int TCS34725_ENABLE_AEN = (0x02);
    /**   Power on - Writing 1 activates the internal oscillator, 0 disables it */
    public static int TCS34725_ENABLE_PON = (0x01);
    /**   Integration time */
    public static int TCS34725_ATIME = (0x01);
    /**   Wait time (if TCS34725_ENABLE_WEN is asserted) */
    public static int TCS34725_WTIME = (0x03);
    /**   WLONG0 = 2.4ms WLONG1 = 0.029s */
    public static int TCS34725_WTIME_2_4MS = (0xFF);
    /**   WLONG0 = 204ms WLONG1 = 2.45s */
    public static int TCS34725_WTIME_204MS = (0xAB);
    /**   WLONG0 = 614ms WLONG1 = 7.4s */
    public static int TCS34725_WTIME_614MS = (0x00);
    /**   Clear channel lower interrupt threshold (lower byte) */
    public static int TCS34725_AILTL = (0x04);
    /**   Clear channel lower interrupt threshold (higher byte) */
    public static int TCS34725_AILTH = (0x05);
    /**   Clear channel upper interrupt threshold (lower byte) */
    public static int TCS34725_AIHTL = (0x06);
    /**   Clear channel upper interrupt threshold (higher byte) */
    public static int TCS34725_AIHTH = (0x07);
    /**
     *   Persistence register - basic SW filtering mechanism for \
     * interrupts
     */
    public static int TCS34725_PERS = (0x0C);
    /**   Every RGBC cycle generates an interrupt */
    public static int TCS34725_PERS_NONE = (0b0000);
    /**
     *   1 clean channel value outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_1_CYCLE = (0b0001);
    /**
     *   2 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_2_CYCLE = (0b0010);
    /**
     *   3 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_3_CYCLE = (0b0011);
    /**
     *   5 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_5_CYCLE = (0b0100);
    /**
     *   10 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_10_CYCLE = (0b0101);
    /**
     *   15 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_15_CYCLE = (0b0110);
    /**
     *   20 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_20_CYCLE = (0b0111);
    /**
     *   25 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_25_CYCLE = (0b1000);
    /**
     *   30 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_30_CYCLE = (0b1001);
    /**
     *   35 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_35_CYCLE = (0b1010);
    /**
     *   40 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_40_CYCLE = (0b1011);
    /**
     *   45 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_45_CYCLE = (0b1100);
    /**
     *   50 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_50_CYCLE = (0b1101);
    /**
     *   55 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_55_CYCLE = (0b1110);
    /**
     *   60 clean channel values outside threshold range generates an \
     * interrupt
     */
    public static int TCS34725_PERS_60_CYCLE = (0b1111);
    /**   Configuration **/
    public static int TCS34725_CONFIG = (0x0D);
    /**
     *   Choose between short and long (12x) wait times via \
     * TCS34725_WTIME
     */
    public static int TCS34725_CONFIG_WLONG = (0x02);
    /**   Set the gain level for the sensor */
    public static int TCS34725_CONTROL = (0x0F);
    /**   0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
    public static int TCS34725_ID = (0x12);
    /**   Device status **/
    public static int TCS34725_STATUS = (0x13);
    /**   RGBC Clean channel interrupt */
    public static int TCS34725_STATUS_AINT = (0x10);
    /**
     *   Indicates that the RGBC channels have completed an integration \
     * cycle
     */
    public static int TCS34725_STATUS_AVALID = (0x01);
    /**   Clear channel data low byte */
    public static int TCS34725_CDATAL = (0x14);
    /**   Clear channel data high byte */
    public static int TCS34725_CDATAH = (0x15);
    /**   Red channel data low byte */
    public static int TCS34725_RDATAL = (0x16);
    /**   Red channel data high byte */
    public static int TCS34725_RDATAH = (0x17);
    /**   Green channel data low byte */
    public static int TCS34725_GDATAL = (0x18);
    /**   Green channel data high byte */
    public static int TCS34725_GDATAH = (0x19);
    /**   Blue channel data low byte */
    public static int TCS34725_BDATAL = (0x1A);
    /**   Blue channel data high byte */
    public static int TCS34725_BDATAH = (0x1B);

    /* Integration time settings for TCS34725 
     *
     * 60-Hz period: 16.67ms, 50-Hz period: 20ms
     * 100ms is evenly divisible by 50Hz periods and by 60Hz periods
     */
    /**   2.4ms - 1 cycle - Max Count: 1024 */
    public static int TCS34725_INTEGRATIONTIME_2_4MS = (0xFF);
    /**   24.0ms - 10 cycles - Max Count: 10240 */
    public static int TCS34725_INTEGRATIONTIME_24MS = (0xF6);
    /**   50.4ms - 21 cycles - Max Count: 21504 */
    public static int TCS34725_INTEGRATIONTIME_50MS = (0xEB);
    /**   60.0ms - 25 cycles - Max Count: 25700 */
    public static int TCS34725_INTEGRATIONTIME_60MS = (0xE7);
    /**   100.8ms - 42 cycles - Max Count: 43008 */
    public static int TCS34725_INTEGRATIONTIME_101MS = (0xD6);
    /**   120.0ms - 50 cycles - Max Count: 51200 */
    public static int TCS34725_INTEGRATIONTIME_120MS = (0xCE);
    /**   153.6ms - 64 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_154MS = (0xC0);
    /**   180.0ms - 75 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_180MS = (0xB5);
    /**   199.2ms - 83 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_199MS = (0xAD);
    /**   240.0ms - 100 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_240MS = (0x9C);
    /**   300.0ms - 125 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_300MS = (0x83);
    /**   360.0ms - 150 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_360MS = (0x6A);
    /**   400.8ms - 167 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_401MS = (0x59);
    /**   420.0ms - 175 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_420MS = (0x51);
    /**   480.0ms - 200 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_480MS = (0x38);
    /**   499.2ms - 208 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_499MS = (0x30);
    /**   540.0ms - 225 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_540MS = (0x1F);
    /**   600.0ms - 250 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_600MS = (0x06);
    /**   614.4ms - 256 cycles - Max Count: 65535 */
    public static int TCS34725_INTEGRATIONTIME_614MS = (0x00);

    private I2C i2c_bus;

    private boolean isInitialized = false;
    private byte integrationTime = 0;
    private TCS34721_GAIN gain = TCS34721_GAIN.TCS34725_GAIN_1X;

    /**
     * Represents a TCS34725 Color sensor.
     */
    public TCS34725() {
        i2c_bus = new I2C(I2C.Port.kMXP, TCS34725.TCS34725_ADDRESS);
    }

    /**
     * Read the RGB color detected by the sensor.
     * 
     * @return TCS34725_RGB
     */
    public TCS34725_RGB getRGB() {

        if (!isInitialized)
            init();

        byte[] buff = new byte[2];
        short r, g, b, c;

        i2c_bus.read(TCS34725_COMMAND_BIT | TCS34725_CDATAL, 2, buff);
        c = bytesToShort(buff);

        i2c_bus.read(TCS34725_COMMAND_BIT | TCS34725_RDATAL, 2, buff);
        r = bytesToShort(buff);

        i2c_bus.read(TCS34725_COMMAND_BIT | TCS34725_GDATAL, 2, buff);
        g = bytesToShort(buff);

        i2c_bus.read(TCS34725_COMMAND_BIT | TCS34725_BDATAL, 2, buff);
        b = bytesToShort(buff);

        TCS34725_RGB rgb;
        if (c == 0) {
            rgb = new TCS34725_RGB(0, 0, 0);
            return rgb;
        }

        rgb = new TCS34725_RGB(r / c * 255, g / c * 255, b / c * 255);

        return rgb;
    }

    /**
     * Initializes I2C and configures the sensor
     * 
     * @param addr
     * i2c address
     * 
     * @param *theWire
     * The Wire object
     * 
     * @return True if initialization was successful, otherwise false.
     */
    private boolean init() {
        // Read from device to see if it's connected
        byte[] buff = new byte[1];
        i2c_bus.read(TCS34725_COMMAND_BIT | 0x12, 1, buff);
        if ((buff[0] != 0x4d) && (buff[0] != 0x44) && (buff[0] != 0x10)) {
            return false;
        }

        isInitialized = true;

        // setGains
        setIntegrationTime(integrationTime);
        setGain(gain);

        enable();

        return true;
    }

    /**
     * Enables the device
     */
    public void enable() {
        i2c_bus.write(TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON);
        i2c_bus.write(TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    }

    /**
     * Disables the device (putting it in lower power sleep mode)
     */
    public void disable() {
        byte[] buff = new byte[1];
        i2c_bus.read(TCS34725_COMMAND_BIT | TCS34725_ENABLE, 1, buff);
        i2c_bus.write(TCS34725_COMMAND_BIT | TCS34725_ENABLE, buff[0] & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
    }

    /**
     * 
     * Sets the integration time for the TC34725
     * 
     * @param time
     * Integration Time
     */
    public void setIntegrationTime(byte time) {
        i2c_bus.write(TCS34725_COMMAND_BIT | TCS34725_ATIME, time);
    }

    /**
     * 
     * Adjusts the gain on the TCS34725
     * 
     * @param gain
     * Gain (sensitivity to light)
     */
    public void setGain(TCS34721_GAIN gain) {
        i2c_bus.write(TCS34725_COMMAND_BIT | TCS34725_CONTROL, gain.value);
    }

    private short bytesToShort(byte[] bytes) {
        if (bytes.length != 2)
            return 0;

        return (short) (bytes[0] << 8 | bytes[1] & 0xFF);
    }

}
