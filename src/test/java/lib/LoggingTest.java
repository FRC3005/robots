package lib;

import static org.junit.Assert.*;

import com.swervetest.LogConfig;
import edu.wpi.first.hal.*;

import org.junit.*;
import org.tinylog.Logger;

public class LoggingTest {
  @Before // this method will run before each test
  public void setup() {
    //assert HAL.initialize(500, 0);
    LogConfig.config(true);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @Test
  public void logTest() {
    Logger.tag("logTest").warn("Test Message");
    Logger.info("Test Message without a tag");
    Logger.trace("Test Message without a tag");
    return;
  }
}
