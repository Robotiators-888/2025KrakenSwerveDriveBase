package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  // Need to publish info to Elastic/NetworkTables
  private static Alert INSTANCE = null;
  private static Elastic.Notification notification = new Elastic.Notification();
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
    Elastic.sendNotification(notification
        .withLevel(Elastic.Notification.NotificationLevel.ERROR)
        .withTitle("Error!")
        .withDescription(alert)
    );
  }
  private void notifyWarning (String alert) {
    Elastic.sendNotification(notification
        .withLevel(Elastic.Notification.NotificationLevel.WARNING)
        .withTitle("Warning:")
        .withDescription(alert)
    );
  }
  private void notifyInfo (String alert) {
    Elastic.sendNotification(notification
        .withLevel(Elastic.Notification.NotificationLevel.INFO)
        .withTitle("Info")
        .withDescription(alert)
    );
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