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

import org.firstinspires.ftc.teamcode.commands.ArmLowGoal;
import org.firstinspires.ftc.teamcode.commands.ArmUp;
import org.firstinspires.ftc.teamcode.commands.ElevatorLowGoal;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pyrolib.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmHighGoal;
import org.firstinspires.ftc.teamcode.commands.ArmLevel;
import org.firstinspires.ftc.teamcode.commands.ElevatorFullIn;
import org.firstinspires.ftc.teamcode.commands.ElevatorHighGoal;
import org.firstinspires.ftc.teamcode.commands.ClawOpen;
import org.firstinspires.ftc.teamcode.commands.ClawHold;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.GamePiece;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class PreloadHigh extends SequentialCommandGroup {

    public PreloadHigh(CommandOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry,
                       Arm arm, Elevator elevator, Claw claw, GamePiece gamePiece) {
        addCommands(
                new ClawHold(claw).withTimeout(500),
                new ArmUp(arm).withTimeout(2000),
                new PreloadHighMove(opMode, hardwareMap, telemetry),
                new ElevatorFullIn(elevator).withTimeout(1000),
                //new Preload(opMode, hardwareMap, telemetry).withTimeout(3000),
                new ArmHighGoal(arm,gamePiece).withTimeout(3000),
                new WaitCommand(500),
                new ElevatorHighGoal(elevator, gamePiece).withTimeout(2000),
                new WaitCommand(500),
                new ClawOpen(claw).withTimeout(500),
                new WaitCommand(500),
                new ElevatorFullIn(elevator).withTimeout(3000),
                new ArmLevel(arm).withTimeout(3000),
                new WaitCommand(30000)
        );
    }

}
