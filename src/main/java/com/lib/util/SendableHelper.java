package com.lib.util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class SendableHelper {
  /**
   * Add a sendable to the dashboard under an existing sendable. The 
   * root sendable must be part of a subsystem.
   * 
   * @param builder Builder passed to the initSendable
   * @param parent Parent sendable object
   * @param child Sendable to add under the parent
   * @param name Name for the group the child will be added under
   */
  static public void addChild(SendableBuilder builder, Sendable parent, Sendable child, String name) {
    String subsystem = SendableRegistry.getSubsystem(parent);
    String parentName = SendableRegistry.getName(parent);
    SendableRegistry.addLW(child, subsystem, parentName + "/" + name);
    child.initSendable(builder);
  }
    
}
