package lib;

import static org.junit.Assert.*;

import org.junit.*;

public class TestTemplateTest {
  @Before // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    // assert HAL.initialize(500, 0);
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @Test
  public void exampleTest() {
    assertTrue("This passes if true", true);

    int myTestValue = 4;
    assertEquals("This passes if these two are equal, expected goes 2nd", 4, myTestValue);
    return;
  }
}
