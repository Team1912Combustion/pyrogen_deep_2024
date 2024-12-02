
package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.OTOS.OTOSSensor;
import org.firstinspires.ftc.teamcode.subsystems.Estimator;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class Screen extends SubsystemBase {

    private final OTOSSensor otos;
    private final Telemetry telemetry;
    private final Estimator estimator;
    private final Odometry odometry;

    public Screen(OTOSSensor o_otos, Odometry o_odometry, Estimator e_estimator, Telemetry t_telemetry) {
        otos = o_otos;
        telemetry = t_telemetry;
        estimator = e_estimator;
        odometry = o_odometry;
    }

    public void periodic() {
        add_telemetry();
    }

    @SuppressLint("DefaultLocale")
    public void add_telemetry() {
        telemetry.addLine(String.format("OTOS XYR %6.1f %6.1f %6.1f",
                otos.getPose2d().getX(),
                otos.getPose2d().getY(),
                otos.getPose2d().getHeading()));
        telemetry.addLine(String.format("Odo XYR %6.1f %6.1f %6.1f",
                odometry.getPose().getX(),
                odometry.getPose().getY(),
                odometry.getPose().getHeading()));
        telemetry.addLine(String.format("Est XYR %6.1f %6.1f %6.1f",
            estimator.getPose().getX(),
            estimator.getPose().getY(),
            estimator.getPose().getHeading()));
    }
}
