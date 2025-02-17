
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.team1912.pyrogen.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.robot.Constants.SpecimenConstants;

public class Specimen extends SubsystemBase {
    private final Servo m_specimen;
    private final Telemetry telemetry;
    private double position;
    private boolean is_open;

    public Specimen(HardwareMap hMap, Telemetry t_telemetry) {
        m_specimen = hMap.get(Servo.class, SpecimenConstants.servo_name);
        telemetry = t_telemetry;
        position = SpecimenConstants.init_pos;
        is_open = false;
        goSafe();
        m_specimen.setPosition(position);
    }

    public void goSafe() {
        position = SpecimenConstants.safe_pos;
    }
    public void goHold() {
        position = SpecimenConstants.hold_pos;
    }
    public void Toggle() {
        if (is_open) {
            is_open = false;
            goHold();
        } else {
            is_open = true;
            goSafe();
        }
    }

    @Override
    public void periodic() {
        m_specimen.setPosition(position);
        telemetry.addLine(String.format("specimen position %f", position));
    }

}
