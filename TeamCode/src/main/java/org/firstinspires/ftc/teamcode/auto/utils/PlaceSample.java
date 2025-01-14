/* Copyright (c) 2024 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto.utils;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;

public class PlaceSample
{

    private boolean I_AM_BLUE;
    private boolean iAmBlue() { return I_AM_BLUE;}

    double driveSpeed = 1.0;
    double slowSpeed = 0.5;
    double minDriveSpeed = 0.25;
    double turnSpeed = 0.20;
    double holdTime = 0.5;

    private AutoDrive autoDrive;

    public void init(AutoDrive m_autodrive, boolean m_iAmBlue) {
        autoDrive = m_autodrive;
        I_AM_BLUE = m_iAmBlue;
    }

    public void update(boolean m_iAmBlue) {
        I_AM_BLUE = m_iAmBlue;
    }

    public void goPushSampleAndReturn(FieldPosition fieldPosition)
    {
        goNet();
    }

    private void goNet() {
        double subHeading = (iAmBlue() ? 90. : -90.);
        double head45 = (iAmBlue() ? 45. : -45.);
        double head135 = (iAmBlue() ? 135. : -135.);
        autoDrive.rampStraight(driveSpeed, minDriveSpeed, 6., 24., 0.0);
        autoDrive.driveStraight(slowSpeed, -12.0, 0.0);
        autoDrive.turnToHeading(turnSpeed, subHeading);
        autoDrive.holdHeading(turnSpeed, subHeading, holdTime);
    }

}
