package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  // Need to publish info to Elastic/NetworkTables
  private static Alert INSTANCE = null;
  // Static is likley not needed
  private static Elastic.Notification notification = new Elastic.Notification(); // Creates one notification object that can be method chained on to increase garbage collection performance
  Vector<String> error;
  Vector<String> warning;
  Vector<String> info;
  Color alertColor;
  String alertString;

  private Alert() {
    alertColor = new Color(0, 255, 0); // Green
    alertString = alertColor.toHexString();
    error = new Vector<String>(new String());
    warning = new Vector<String>(new String());
    info = new Vector<String>(new String());
    SmartDashboard.putString("Alerts", alertString);    
    updateSmartDashboard();
    // testcalls();
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
    updateSmartDashboard();
  }

  public void registerWarning (String alert) {
    warning.add(alert);
    registerColor();
    updateSmartDashboard();
  }

  public void registerInfo (String alert) {
    info.add(alert);
    registerColor();
    updateSmartDashboard();
  }

  public void notifyError (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.ERROR)
      .withTitle("Error!")
      .withDescription(alert)
    );
  }

  public void notifyWarning (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.WARNING)
      .withTitle("Warning:")
      .withDescription(alert)
    );
  }

  public void notifyInfo (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.INFO)
      .withTitle("Info")
      .withDescription(alert)
    );
  }

  // Sets the single color elastic object to the highes severity level that the robot has (check engine light)
  private void registerColor () {
    if (!error.isEmpty()) {
      alertColor = new Color(255, 0, 0); // Red
    }
    else if (!warning.isEmpty()) {
      alertColor = new Color(255, 255, 0); // Yellow
    }
    else {
      alertColor = new Color(0, 255, 0); // Green
    }
    alertString = alertColor.toHexString();
  }

  public void triggerStop () { // Stops the robot from running with errors
    // Todo: implement this
  }

  private void testcalls () {
    registerWarning("There may be an issue");
    notifyWarning("Photonvision has optional type idk");
  }

  private void updateSmartDashboard () {
    // ArrayList required aquark casting to work so I used Vector
    SmartDashboard.putStringArray("errors", error.toArray());
    SmartDashboard.putStringArray("warnings", warning.toArray());
    SmartDashboard.putStringArray("info", info.toArray());   
  }
}