package lib;

import static org.junit.Assert.*;

import java.util.function.Function;

import frc.lib.vendor.motorcontroller.SparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.junit.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;

public class SparkMaxTest {
    
    @Before
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @After
    public void shutdown() throws Exception {

    }

    private static final Function<CANSparkMax, Boolean> initFunction = (CANSparkMax sparkMax) -> {
        sparkMax.restoreFactoryDefaults();
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(20);
        sparkMax.burnFlash();

        sparkMax.enableVoltageCompensation(12.12);
        return true;
    };

    @Test
    public void basicDriverTest() {
        SparkMax sm = new SparkMax(
            new CANSparkMax(4, MotorType.kBrushless),
            initFunction
        );

        int devId = sm.getCANSparkMax().getDeviceId();
        var tmp = sm.getCANSparkMax().getIdleMode();

        assertEquals(4, devId);

        assertEquals(IdleMode.kBrake, tmp);

        sm.mutate(spark -> {
            spark.setIdleMode(IdleMode.kCoast);
            return true;
        });

        tmp = sm.getCANSparkMax().getIdleMode();
        assertEquals(IdleMode.kCoast, tmp);
    }
}
