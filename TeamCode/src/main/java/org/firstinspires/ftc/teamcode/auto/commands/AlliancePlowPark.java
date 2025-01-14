/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;

public class AlliancePlowPark extends CommandBase {

    AutoDriveHelpers autodrive;
    HardwareMap hMap;
    boolean amIFinished = false;
    double turnSpeed = 0.;
    double driveSpeed = 0.;
    double head = 0.;

    public AlliancePlowPark(CommandOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        autodrive = new AutoDriveHelpers(opMode, telemetry);
        hMap = hardwareMap;
    }

    @Override
    public void execute() {
        amIFinished = false;
        autodrive.init(hMap);
        turnSpeed = autodrive.TURN_SPEED*1.5;
        driveSpeed = autodrive.DRIVE_SPEED*1.5;
        double holdTime = 0.25;

        head = 0.;
        autodrive.strafeStraight(driveSpeed, -6.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);

        head = 90.;
        autodrive.turnToHeading(turnSpeed, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);

        // block 1
        autodrive.driveStraight(driveSpeed, 48.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.strafeStraight(driveSpeed, 14.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.driveStraight(driveSpeed, -48.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        // block 2
        autodrive.driveStraight(driveSpeed, 48.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.strafeStraight(driveSpeed, 12.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.driveStraight(driveSpeed, -48.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        // block 3
        autodrive.driveStraight(driveSpeed, 48.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.strafeStraight(driveSpeed, 12.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        autodrive.driveStraight(driveSpeed, -44.0, head);
        autodrive.holdHeading( turnSpeed, head, holdTime);
        amIFinished = true;
    }

    @Override
    public boolean isFinished() {
        return amIFinished;
    }

}
