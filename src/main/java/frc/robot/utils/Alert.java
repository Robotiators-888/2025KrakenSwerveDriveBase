package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  // Need to publish info to Elastic/NetworkTables
  private static Alert INSTANCE = null;
  ArrayList<String> error;
  ArrayList<String> warning;
  ArrayList<String> info;
  private Alert() {
    error = new ArrayList<String>();
    warning = new ArrayList<String>();
    info = new ArrayList<String>();
  }
  public static Alert getInstance () {
    if (INSTANCE == null) {
      INSTANCE = new Alert();
    }
    return INSTANCE;
  }
  public void registerError (String alert) {
    error.add(alert);
    registerColor();
  }
  public void registerWarning (String alert) {
    warning.add(alert);
    registerColor();
  }
  public void registerInfo (String alert) {
    info.add(alert);
    registerColor();
  }
  private void notifyError (String alert) {
    Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Error!", alert);
    Elastic.sendNotification(notification);
  }
  private void notifyWarning (String alert) {
    Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Error!", alert);
    Elastic.sendNotification(notification);
  }
  private void notifyInfo (String alert) {
    Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "Error!", alert);
    Elastic.sendNotification(notification);
  }
  // Sets the single color elastic object to the highes severity level that the robot has (check engine light)
  private void registerColor () {
    Color exampleColor;
    if (!error.isEmpty()) {
        exampleColor = new Color(255, 0, 0); // Red
    }
    else if (!warning.isEmpty()) {
        exampleColor = new Color(255, 255, 0); // Yellow
    }
    else {
        exampleColor = new Color(0, 255, 0); // Green
    }
    // Not sure why this is SmartDashboard if this is elastic
    SmartDashboard.putString("Alerts", exampleColor.toHexString());        
  }
  public void triggerStop () {
    // Todo: implement this
  }
}