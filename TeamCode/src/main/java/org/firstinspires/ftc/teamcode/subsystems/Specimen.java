
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.Constants.SpecimenConstants;

public class Specimen extends SubsystemBase {
    private final Servo m_specimen;
    private final Telemetry telemetry;
    private double position;

   /*
    public static final String servo_name = "specimen";
    public static final double init_pos = 0.0;
    public static final double safe_pos = 0.0;
    public static final double hold_pos = 0.5;
    */

    public Specimen(HardwareMap hMap, Telemetry t_telemetry) {
        m_specimen = hMap.get(Servo.class, SpecimenConstants.servo_name);
        telemetry = t_telemetry;
        position = SpecimenConstants.init_pos;
        m_specimen.setPosition(position);
    }

    public void goSafe() {
        position = SpecimenConstants.safe_pos;
    }
    public void goHold() {
        position = SpecimenConstants.hold_pos;
    }

    @Override
    public void periodic() {
        m_specimen.setPosition(position);
        telemetry.addLine(String.format("specimen position %f\n", position));
    }

}
