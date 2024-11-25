
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.CRServo;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.robot.Constants;

public class Intake extends SubsystemBase {
    private final CRServo m_intake;

    public Intake(HardwareMap hMap, String servoName) {
        m_intake = new CRServo(hMap, servoName);
    }

    public void runIn() {
        m_intake.set(Constants.IntakeConstants.intake_speed);
    }
    public void runOut() {
        m_intake.set(Constants.IntakeConstants.output_speed);
    }

    public void stop() {
        m_intake.set(0.);
    }
}
