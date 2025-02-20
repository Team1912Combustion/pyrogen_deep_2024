package org.team1912.pyrogen.pyrolib.stampede;

import org.team1912.pyrogen.pyrolib.GoBildaPinpoint.PinpointSensor;

public class AngleTrackerPinpoint {

    private PinpointSensor pinpoint;      // Control Hub IMU
    private double resetVal = 0;

    public AngleTrackerPinpoint(PinpointSensor pinpoint) {
            this.pinpoint = pinpoint;

        resetOrientation();
    }

    public double getHeading() {
        return pinpoint.getHeading()*180./Math.PI;
    }

    public double getOrientation() {
        double val = getHeading() - resetVal;  //Gets angle in degrees (negatives and over 360 included)
        val = ((val % 360) + 360) % 360; //make value between 0-360 degrees

        return val;
    }

    public void resetOrientation() {
        resetVal = getHeading();
    }

    public void setOrientation(double heading) {
        resetOrientation();
        resetVal -= heading;
    }
}
