/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * This package is started from 
 * https://github.com/juchong/ADIS16470-RoboRIO-Driver
 * commit bcb6ed6ac4d323bc329bdd1b65342f6792dd1256
 * 
 * Major changes from the above:
 * 
 * - Only pull data relevent to yaw (removes complimentary filter and inclinometer)
 * - Remove all threading since it is not actually needed, assuming the user calls
 * the update function frequently enough
 * - Adds an update function to run the gyro calculatuon instead of a thread
 * - Removes some of the configuration API since it makes no sense to changes these
 * after initializaiton (e.g. setting a new yaw axis)
 * - Changes the initialization and calibration functions to the following steps:
 * 
 * 1) Initialize everything but do not start auto-spi or calibrate
 * 2) Allow user to call a new start() function which kicks off the calibration and
 * starts auto-spi
 * 3) Sensor config is now in an immutable state, unless the sensor suffers a major
 * power incident. This means the sensor no longer changes between auto and standard
 * SPI modes.
 * 
 * The motivation here is to simplify the code after initialization, and to allow
 * longer initialization times. This matches the way FRC events are conducted. The
 * user does not call start() on the sensor until the beginning of auto. The sensor
 * automatically uses the *past* data to calibrate, and after only a 50ms delay the
 * sensor is ready to go. This is perfect for the FRC use case, since the robot is
 * typically on the field sitting idle for a good amount of time before auton begins.
 * The user no longer needs to worry about not touching the robot when first powering
 * on, or setting code etc. and the calibration time can be longer.
 */

package frc.lib.vendor.sensor;

import java.lang.reflect.Array;
//import java.lang.FdLibm.Pow;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.sql.Driver;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.tinylog.Logger;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.util.RunLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is for the ADIS16470 IMU that connects to the RoboRIO SPI port.
 */
@SuppressWarnings("unused")
public class ADIS16470 implements Gyro, Sendable {
  
  /* ADIS16470 Register Map Declaration */
  private static final int FLASH_CNT      =   0x00;  //Flash memory write count
  private static final int DIAG_STAT      =   0x02;  //Diagnostic and operational status
  private static final int X_GYRO_LOW     =   0x04;  //X-axis gyroscope output, lower word
  private static final int X_GYRO_OUT     =   0x06;  //X-axis gyroscope output, upper word
  private static final int Y_GYRO_LOW     =   0x08;  //Y-axis gyroscope output, lower word
  private static final int Y_GYRO_OUT     =   0x0A;  //Y-axis gyroscope output, upper word
  private static final int Z_GYRO_LOW     =   0x0C;  //Z-axis gyroscope output, lower word
  private static final int Z_GYRO_OUT     =   0x0E;  //Z-axis gyroscope output, upper word
  private static final int X_ACCL_LOW     =   0x10;  //X-axis accelerometer output, lower word
  private static final int X_ACCL_OUT     =   0x12;  //X-axis accelerometer output, upper word
  private static final int Y_ACCL_LOW     =   0x14;  //Y-axis accelerometer output, lower word
  private static final int Y_ACCL_OUT     =   0x16;  //Y-axis accelerometer output, upper word
  private static final int Z_ACCL_LOW     =   0x18;  //Z-axis accelerometer output, lower word
  private static final int Z_ACCL_OUT     =   0x1A;  //Z-axis accelerometer output, upper word
  private static final int TEMP_OUT       =   0x1C;  //Temperature output (internal, not calibrated)
  private static final int TIME_STAMP     =   0x1E;  //PPS mode time stamp
  private static final int DATA_CNTR      =   0x22;  //Data counter
  private static final int X_DELTANG_LOW  =   0x24;  //X-axis delta angle output, lower word
  private static final int X_DELTANG_OUT  =   0x26;  //X-axis delta angle output, upper word
  private static final int Y_DELTANG_LOW  =   0x28;  //Y-axis delta angle output, lower word
  private static final int Y_DELTANG_OUT  =   0x2A;  //Y-axis delta angle output, upper word
  private static final int Z_DELTANG_LOW  =   0x2C;  //Z-axis delta angle output, lower word
  private static final int Z_DELTANG_OUT  =   0x2E;  //Z-axis delta angle output, upper word
  private static final int X_DELTVEL_LOW  =   0x30;  //X-axis delta velocity output, lower word
  private static final int X_DELTVEL_OUT  =   0x32;  //X-axis delta velocity output, upper word
  private static final int Y_DELTVEL_LOW  =   0x34;  //Y-axis delta velocity output, lower word
  private static final int Y_DELTVEL_OUT  =   0x36;  //Y-axis delta velocity output, upper word
  private static final int Z_DELTVEL_LOW  =   0x38;  //Z-axis delta velocity output, lower word
  private static final int Z_DELTVEL_OUT  =   0x3A;  //Z-axis delta velocity output, upper word
  private static final int XG_BIAS_LOW    =   0x40;  //X-axis gyroscope bias offset correction, lower word
  private static final int XG_BIAS_HIGH   =   0x42;  //X-axis gyroscope bias offset correction, upper word
  private static final int YG_BIAS_LOW    =   0x44;  //Y-axis gyroscope bias offset correction, lower word
  private static final int YG_BIAS_HIGH   =   0x46;  //Y-axis gyroscope bias offset correction, upper word
  private static final int ZG_BIAS_LOW    =   0x48;  //Z-axis gyroscope bias offset correction, lower word
  private static final int ZG_BIAS_HIGH   =   0x4A;  //Z-axis gyroscope bias offset correction, upper word
  private static final int XA_BIAS_LOW    =   0x4C;  //X-axis accelerometer bias offset correction, lower word
  private static final int XA_BIAS_HIGH   =   0x4E;  //X-axis accelerometer bias offset correction, upper word
  private static final int YA_BIAS_LOW    =   0x50;  //Y-axis accelerometer bias offset correction, lower word
  private static final int YA_BIAS_HIGH   =   0x52;  //Y-axis accelerometer bias offset correction, upper word
  private static final int ZA_BIAS_LOW    =   0x54;  //Z-axis accelerometer bias offset correction, lower word
  private static final int ZA_BIAS_HIGH   =   0x56;  //Z-axis accelerometer bias offset correction, upper word
  private static final int FILT_CTRL      =   0x5C;  //Filter control
  private static final int MSC_CTRL       =   0x60;  //Miscellaneous control
  private static final int UP_SCALE       =   0x62;  //Clock scale factor, PPS mode
  private static final int DEC_RATE       =   0x64;  //Decimation rate control (output data rate)
  private static final int NULL_CNFG      =   0x66;  //Auto-null configuration control
  private static final int GLOB_CMD       =   0x68;  //Global commands
  private static final int FIRM_REV       =   0x6C;  //Firmware revision
  private static final int FIRM_DM        =   0x6E;  //Firmware revision date, month and day
  private static final int FIRM_Y         =   0x70;  //Firmware revision date, year
  private static final int PROD_ID        =   0x72;  //Product identification 
  private static final int SERIAL_NUM     =   0x74;  //Serial number (relative to assembly lot)
  private static final int USER_SCR1      =   0x76;  //User scratch register 1 
  private static final int USER_SCR2      =   0x78;  //User scratch register 2 
  private static final int USER_SCR3      =   0x7A;  //User scratch register 3 
  private static final int FLSHCNT_LOW    =   0x7C;  //Flash update count, lower word 
  private static final int FLSHCNT_HIGH   =   0x7E;  //Flash update count, upper word 

  private static final byte[] m_autospi_x_packet = {
    X_DELTANG_OUT,
    FLASH_CNT,
    X_DELTANG_LOW,
    FLASH_CNT,
    DATA_CNTR,
    FLASH_CNT,
    DIAG_STAT,
    FLASH_CNT,
  };

  private static final byte[] m_autospi_y_packet = {
    Y_DELTANG_OUT,
    FLASH_CNT,
    Y_DELTANG_LOW,
    FLASH_CNT,
    DATA_CNTR,
    FLASH_CNT,
    DIAG_STAT,
    FLASH_CNT,
  };

  private static final byte[] m_autospi_z_packet = {
    Z_DELTANG_OUT,
    FLASH_CNT,
    Z_DELTANG_LOW,
    FLASH_CNT,
    DATA_CNTR,
    FLASH_CNT,
    DIAG_STAT,
    FLASH_CNT,
  };

  public enum IMUAxis { kX, kY, kZ }

  public enum ADIS16470CalibrationTime { 
    _32ms(0), _64ms(1), _128ms(2), _256ms(3),_512ms(4), _1s(5), _2s(6), _4s(7), _8s(8), _16s(9), _32s(10), _64s(11);
    private int value;
    private ADIS16470CalibrationTime(int value) {
      this.value = value;
    }
  }

  // Static Constants
  private static final double delta_angle_sf = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  private static final double rad_to_deg = 57.2957795;
  private static final double deg_to_rad = 0.0174532;

  // Add 1 for timestamp from DMA
  private final int kDataLength = m_autospi_z_packet.length+ 1;
  private final int kBufferSize = 4000;
  private int[] m_autoSPIBuffer = new int[kBufferSize];

  // User-specified yaw axis
  private IMUAxis m_yaw_axis;

  // Integrated gyro angle
  private double m_integ_angle = 0.0;

  // State variables
  private int m_calibration_time = 0;
  private volatile boolean m_first_run = true;
  private double m_scaled_sample_rate = 2500.0;
  private boolean m_auto_configured = false;
  private final double m_startTime;
  private int m_dataCount;
  private int m_dataCountErrors = 0;

  // Resources
  private SPI m_spi;
  private SPI.Port m_spi_port;
  private DigitalInput m_auto_interrupt;
  private DigitalOutput m_reset_out;
  private DigitalInput m_reset_in;
  private DigitalOutput m_status_led;

  // Previous timestamp
  long previous_timestamp = 0;

  /**
   * @param yaw_axis Which axis is Yaw
   * @param port     SPI port to use
 * @throws Exception
   */
  public ADIS16470(IMUAxis yaw_axis, SPI.Port port, ADIS16470CalibrationTime cal_time) throws Exception {
    m_yaw_axis = yaw_axis;
    m_calibration_time = cal_time.value;
    m_spi_port = SPI.Port.kOnboardCS0;

    // The start time is likely close to when the sensor is first powered on, use this later to make sure
    // enough total time has passed for calibration.
    m_startTime = Timer.getFPGATimestamp();

    // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
    // Relies on the RIO hardware by default configuring an output as low
    // and configuring an input as high Z. The 10k pull-up resistor internal to the
    // IMU then forces the reset line high for normal operation.
    m_reset_out = new DigitalOutput(27); // Drive SPI CS2 (IMU RST) low
    Timer.delay(0.01); // Wait 10ms
    m_reset_out.close();
    m_reset_in = new DigitalInput(27); // Set SPI CS2 (IMU RST) high
    Timer.delay(0.25); // Wait 250ms for reset to complete

    // Configure Data-Ready pin on SPI CS1
    m_auto_interrupt = new DigitalInput(26);

    for (int i = 0; !initializeSPI(); i++) {
        // Retry in 1 second
        Timer.delay(1);

        if (i == 5) {
            throw new Exception("Unable to initialize ADIS16470");
        }
    }

    // Set IMU internal decimation to 100 SPS (output data rate of 2000 SPS / (19 + 1) = 100Hz)
    writeRegister(DEC_RATE, 0x13);

    // Set data ready polarity (raising), Disable gSense Compensation and PoP
    writeRegister(MSC_CTRL, 0xC1);

    // Disable IMU internal Bartlett filter
    writeRegister(FILT_CTRL, 0);

    // Configure continuous bias calibration time based on user setting
    writeRegister(NULL_CNFG, (m_calibration_time | 0x0700));
    
    /* TODO: Delete this
    // Write offset calibration command to IMU
    writeRegister(GLOB_CMD, 0x1);
    Timer.delay(0.05); // Wait 50ms GLOB_CMD timing


    // Configure and enable auto SPI
    for (int i = 0; !switchToAutoSPI(); i++) {
        // Retry in 1 second
        Timer.delay(1);

        if (i == 5) {
            throw new Exception("Unable to enable auto-SPI and initialize ADIS16470");
        }
    }
    */

    // Let the user know the IMU was initiallized successfully
    Logger.tag("ADIS16470").debug("Successfully Initialized, waiting to start...");

    // Drive "Ready" LED to 'waiting to start' state
    m_status_led = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low
    // Only 1 PWM frequency for the entire set of DIO, so this may need to change...
    m_status_led.setPWMRate(0.6);
    m_status_led.enablePWM(0.5);

    // Report usage and post data to DS
    HAL.report(tResourceType.kResourceType_ADIS16470, 0);
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toUShort(ByteBuffer buf) {
    return (buf.getShort(0)) & 0xFFFF;
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toUShort(byte[] buf) {
    return (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }  

  /**
   * 
   * @param data
   * @return
   */
  private static int toUShort(int... data) {
	  byte[] buf = new byte[data.length];
	  for(int i = 0; i < data.length; ++i) {
		  buf[i] = (byte)data[i];
	  }
	  return toUShort(buf);
  }

  /**
   * 
   * @param sint
   * @return
   */
  private static long toULong(int sint) {
    return sint & 0x00000000FFFFFFFFL;
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toShort(int... buf) {
    return (short)(((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toShort(ByteBuffer buf) {
	  return toShort(buf.get(0), buf.get(1));
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toShort(byte[] buf) {
    return buf[0] << 8 | buf[1];
  }

  /**
   * 
   * @param buf
   * @return
   */
  private static int toInt(int... buf) {
    return (int)((buf[0] & 0xFF) << 24 | (buf[1] & 0xFF) << 16 | (buf[2] & 0xFF) << 8 | (buf[3] & 0xFF));
  }

  /**
   * Initialize the SPI port
   * 
   * @return true if the port initialized successfully
   */
  private boolean initializeSPI() {
    if (m_spi == null) {
        Logger.tag("ADIS16470").debug("Initializing SPI port.");
        m_spi = new SPI(m_spi_port);
    } else {
        Logger.tag("ADIS16470").debug("Re-initializing SPI port.");
        m_spi.close();
        m_spi = new SPI(m_spi_port);
    }

    m_spi.setClockRate(2000000);
    m_spi.setMSBFirst();
    m_spi.setSampleDataOnTrailingEdge();
    m_spi.setClockActiveLow();
    m_spi.setChipSelectActiveLow();
    readRegister(PROD_ID); // Dummy read (but why??)

    // Validate the product ID
    if (readRegister(PROD_ID) != 16982) {
        Logger.tag("ADIS16470").error("Could not find ADIS16470");
        m_spi.close();
        m_spi = null;
        return false;
    }

    return true;
  }

  /**
   * Switch to standard SPI mode.
   * 
   * @return
   */
  private boolean switchToStandardSPI() {
    // TODO: Remove this method...
    // There doesn't seem to be a SPI port active. Let's try to set one up
    if (m_spi != null) {
        Logger.tag("ADIS16470").info("Switching to Standard SPI (shouldn't need to do this).");
        // Maybe we're in auto SPI mode? If so, kill auto SPI, and then SPI.
        if (m_auto_configured) {
            m_spi.stopAuto();
            // We need to get rid of all the garbage left in the auto SPI buffer after stopping it.
            // Sometimes data magically reappears, so we have to check the buffer size a couple of times
            //  to be sure we got it all. Yuck.
            int[] trashBuffer = new int[200];
            try{Thread.sleep(100);}catch(InterruptedException e){}
            int data_count = m_spi.readAutoReceivedData(trashBuffer, 0, 0);
            while (data_count > 0) {
                m_spi.readAutoReceivedData(trashBuffer, Math.min(data_count, 200), 0);
                data_count = m_spi.readAutoReceivedData(trashBuffer, 0, 0);
            }
            Logger.tag("ADIS16470").debug("Paused auto SPI successfully.");
        }
        return true;
    } else {
        Logger.tag("ADIS16470").info("Unable to switch to standard SPI, in auto-SPI.");
        return false;
    }
  }

  /**
   * 
   * @return
   */
  boolean startAutoSPI() {
    // No SPI port has been set up. Go set one up first.
    if (m_spi == null) {
        Logger.tag("ADIS16470").error("SPI not initialized to start auto SPI");
        return false;
    }

    // The auto SPI controller gets angry if you try to set up two instances on one bus.
    if (!m_auto_configured) {
      m_spi.initAuto(kBufferSize);
      m_auto_configured = true;
    }

    switch (m_yaw_axis) {
    case kX:
      m_spi.setAutoTransmitData(m_autospi_x_packet, 0);
      break;
    case kY:
      m_spi.setAutoTransmitData(m_autospi_y_packet, 0);
      break;
    case kZ:
      m_spi.setAutoTransmitData(m_autospi_z_packet, 0);
      break;
    default:
      Logger.tag("ADIS16470").error("Invalid yaw axis setting {}", m_yaw_axis);
      return false;
    }

    // ADIS16470 needs the cs pin to go low for some time between reads
    m_spi.configureAutoStall(5, 640, 1);

    // Kick off auto SPI (Note: Device configration impossible after auto SPI is activated)
    // DR High = Data good (data capture should be triggered on the rising edge)
    m_spi.startAutoTrigger(m_auto_interrupt, true, false);

    Logger.tag("ADIS16470").debug("AutoSPI started.");
    return true;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void calibrate() {
      // Do nothing, the user must call start, this override is confusing otherwise.
      Logger.tag("ADIS16470").warn("No need to call calibrate(), calibration is done when calling start()");
  }

  /**
   * 
   * @param reg
   * @return
   */
  private int readRegister(int reg) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    buf.order(ByteOrder.BIG_ENDIAN);
    buf.put(0, (byte) (reg & 0x7f));
    buf.put(1, (byte) 0);

    m_spi.write(buf, 2);
    m_spi.read(false, buf, 2);

    return toUShort(buf);
  }

  /**
   * 
   * @param reg
   * @param val
   */
  private void writeRegister(int reg, int val) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    // low byte
    buf.put(0, (byte) ((0x80 | reg)));
    buf.put(1, (byte) (val & 0xff));
    m_spi.write(buf, 2);
    // high byte
    buf.put(0, (byte) (0x81 | reg));
    buf.put(1, (byte) (val >> 8));
    m_spi.write(buf, 2);
  }

  /**
   * {@inheritDoc}
   */
  public void reset() {
    m_integ_angle = 0.0;
  }

  /**
   * Delete (free) the spi port used for the IMU.
   */
  @Override
  public void close() {
    if (m_spi != null) {
        if (m_auto_configured) {
            m_spi.stopAuto();
        }
        m_spi.close();
        m_auto_configured = false;
        if (m_auto_interrupt != null) {
            m_auto_interrupt.close();
            m_auto_interrupt = null;
        }
        m_spi = null;
    }
  }

  /**
   * 
   */
  private void acquire() {
    int bytesAvailable = m_spi.readAutoReceivedData(m_autoSPIBuffer, 0, 0); // Read number of bytes currently stored in the buffer
    int framesAvailable = (int)(bytesAvailable / kDataLength);

    int overflow = m_spi.getAutoDroppedCount();
    if(overflow > 0) {
        // TODO: Should this cleanup and restart the buffer?
        Logger.tag("ADIS16470").error("Unhandled AutoSPI buffer overrun, dropped count: {}", overflow);
    }

    m_spi.readAutoReceivedData(m_autoSPIBuffer, framesAvailable * kDataLength, 0);
    
    // Could be multiple data sets in the buffer. Handle each one.
    for (int i = 0; i < framesAvailable; i += kDataLength) { 
        // Timestamp is at buffer[i]
        double dt = ((double)(m_autoSPIBuffer[i] - previous_timestamp)) / 1000000.0;
        int dataCount = toUShort(m_autoSPIBuffer[i + 7], m_autoSPIBuffer[i + 8]);
        int status = 0;

        /* Get delta angle value for selected yaw axis and scale by the elapsed time (based on timestamp) */
        double delta_angle = (toInt(m_autoSPIBuffer[i + 3], m_autoSPIBuffer[i + 4], m_autoSPIBuffer[i + 5], m_autoSPIBuffer[i + 6]) * delta_angle_sf) /
                      (m_scaled_sample_rate / (m_autoSPIBuffer[i] - previous_timestamp));

        previous_timestamp = m_autoSPIBuffer[i];

        if(m_first_run) {
            // Throw our first sample which is too stale
            m_integ_angle = 0.0;
            m_dataCount = dataCount;
        } else {
            m_integ_angle += delta_angle;
            if (dataCount != ((m_dataCount + 1) & 0xFFFF)) {
                Logger.tag("ADIS16470").error("Data count failed! Expected {} got {}", m_dataCount + 1, dataCount);
                m_dataCountErrors += 1;
            }
            m_dataCount = dataCount;
        }
        m_first_run = false;
    }
  }

  /**
   * 
   * @param compAngle
   * @param accAngle
   * @return
   */
  private double formatFastConverge(double compAngle, double accAngle) {
    if(compAngle > accAngle + Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    else if (accAngle > compAngle + Math.PI) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle; 
  }

  /**
   * 
   * @param compAngle
   * @return
   */
  private double formatRange0to2PI(double compAngle) {
    while(compAngle >= 2 * Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    while(compAngle < 0.0) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /**
   * 
   * @param accelAngle
   * @param accelZ
   * @return
   */
  private double formatAccelRange(double accelAngle, double accelZ) {
    if(accelZ < 0.0) {
      accelAngle = Math.PI - accelAngle;
    }
    else if(accelZ > 0.0 && accelAngle < 0.0) {
      accelAngle = 2.0 * Math.PI + accelAngle;
    }
    return accelAngle;
  }

  /**
   * {@inheritDoc}
   */
  public double getAngle() {
    return m_integ_angle;
  }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", () -> getAngle(), null);
    }

    @Override
    public double getRate() {
        RunLimiter.once(() -> Logger.tag("ADIS16470").error("getRate() not implemented"));
        return 0;
    }
}