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
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandBase;
import org.team1912.pyrogen.pyrolib.ftclib.command.CommandOpMode;

public class PreloadSamplePrep extends CommandBase {

    AutoDriveHelpers autodrive;
    HardwareMap hMap;
    boolean amIFinished;

    public PreloadSamplePrep(CommandOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        autodrive = new AutoDriveHelpers(opMode, telemetry);
        hMap = hardwareMap;
    }

    @Override
    public void execute() {
        amIFinished = false;
        autodrive.init(hMap);

        autodrive.strafeStraight(autodrive.DRIVE_SPEED, 30.0, 0.0);
        autodrive.holdHeading( autodrive.TURN_SPEED, 0.0, 1.0);

        autodrive.turnToHeading(autodrive.TURN_SPEED, 45.0);
        autodrive.holdHeading( autodrive.TURN_SPEED, 45.0, 1.0);
        autodrive.driveStraight(autodrive.DRIVE_SPEED, 30.0, 45.0);
        autodrive.holdHeading( autodrive.TURN_SPEED, 45.0, 1.0);
        autodrive.moveRobot(0.,0.);
        amIFinished = true;
    }

    @Override
    public boolean isFinished() {
            return amIFinished;
    }

}
