/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

public class NotTeleopBasicOpMode_Linear {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftMotorFront = null;
    public DcMotor rightMotorFront = null;
    public DcMotor leftMotorBack = null;
    public DcMotor rightMotorBack = null;
    public DcMotor arm = null;
    public Servo claw = null;


    public final static double CLAW_HOME = 0.5; // Starting position
    public final static double CLAW_MIN_RANGE = 0.2; // Smallest number allowed for servo position
    public final static double CLAW_MAX_RANGE = 0.8; // Largest number allowed for servo position

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        leftMotorFront = hwMap.dcMotor.get("fl");
        leftMotorBack = hwMap.dcMotor.get("lb");
        rightMotorFront = hwMap.dcMotor.get("fr");
        rightMotorBack = hwMap.dcMotor.get("rb");
        arm = hwMap.dcMotor.get("arm");
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);

        leftMotorBack.setPower(0);
        leftMotorFront.setPower(0);
        rightMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        arm.setPower(0);

        claw = hwMap.servo.get("claw");
        claw.setPosition(CLAW_HOME);


    }
}

