package lib;

import static org.junit.Assert.*;

import java.util.Iterator;

import frc.lib.util.AccessOrderHashSet;

import org.junit.*;

public class AccessOrderHashSetTest {
  @Before // this method will run before each test
  public void setup() {
    // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @Test
  public void insertTest() {
    AccessOrderHashSet<Integer> hs = new AccessOrderHashSet<>();
    hs.add(1);
    hs.add(10);
    hs.add(5);
    Iterator it = hs.iterator();

    assertEquals(1, it.next());
    assertEquals(10, it.next());
    assertEquals(5, it.next());
    return;
  }

  @Test
  public void insertAccessTest() {
    AccessOrderHashSet<Integer> hs = new AccessOrderHashSet<>();
    hs.add(1);
    hs.add(10);
    hs.add(5);
    hs.add(10);

    Iterator it = hs.iterator();
    assertEquals(1, it.next());
    assertEquals(5, it.next());
    assertEquals(10, it.next());
    assertEquals(3, hs.size());
    return;
  }
}
