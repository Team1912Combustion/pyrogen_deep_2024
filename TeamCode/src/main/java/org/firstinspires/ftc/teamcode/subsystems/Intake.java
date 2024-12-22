
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.hardware.motors.CRServo;
import org.firstinspires.ftc.teamcode.robot.Constants;

public class Intake extends SubsystemBase {
    private final CRServo m_intake;
    private final Telemetry telemetry;
    private double speed;

    public Intake(HardwareMap hMap, String servoName, Telemetry t_telemetry) {
        m_intake = new CRServo(hMap, servoName);
        telemetry = t_telemetry;
        speed = 0.;
        m_intake.set(speed);
    }

    public void runIn() {
        speed = Constants.IntakeConstants.intake_speed;
    }
    public void runOut() {
        speed = Constants.IntakeConstants.output_speed;
    }
    public void stop() {
        speed = 0.;
    }

    @Override
    public void periodic() {
        m_intake.set(speed);
        telemetry.addLine(String.format("intake speed %f\n", speed));
    }

}
