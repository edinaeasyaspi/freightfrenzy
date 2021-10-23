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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Red_Auto_1", group="Linear Opmode")
@Disabled
public class Red_Auto_1 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device



    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotor2;
    DcMotor rightMotor2;

    double power = 0.5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        leftMotor2 = hardwareMap.dcMotor.get("Left_Motor2");
        rightMotor2 = hardwareMap.dcMotor.get("Right_Motor2");


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Turn left
        leftMotor.setPower(-power);
        leftMotor2.setPower(power);
        rightMotor.setPower(power);
        rightMotor2.setPower(-power);

        sleep(1000);

        //Go forward and spin the wheel
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor2.setPower(power);
        rightMotor2.setPower(power);

        sleep(500);

        //Go backwards
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        leftMotor2.setPower(-power);
        rightMotor2.setPower(-power);

        sleep(500);

        //Turn 180 degrees
        leftMotor.setPower(-power);
        leftMotor2.setPower(power);
        rightMotor.setPower(power);
        rightMotor2.setPower(-power);

        sleep(2000);

        //Go forward and park
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor2.setPower(power);
        rightMotor2.setPower(power);

        sleep(3000);

        power = 0.0;

        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor2.setPower(power);
        rightMotor2.setPower(power);
        }
    }

