package frc.robot.utils;

import java.util.ArrayList;

public class Alerts {
    private static Alerts INSTANCE = null;
    ArrayList<String> stop;
    ArrayList<String> warning;
    ArrayList<String> info;
    private Alerts() {
        stop = new ArrayList<String>();
        warning = new ArrayList<String>();
        info = new ArrayList<String>();
    }
    public static Alerts getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new Alerts();
        }
        return INSTANCE;
    }
    public void registerStop (String alert) {
        stop.add(alert);
    }
    public void registerWarning (String alert) {
        warning.add(alert);
    }
    public void registerInfo (String alert) {
        info.add(alert);
    }
    public void triggerStop () {} // Need to implement this
}