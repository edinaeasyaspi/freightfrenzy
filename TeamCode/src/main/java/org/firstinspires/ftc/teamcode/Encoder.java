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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Encoder_Auto", group="Linear Opmode")
//@Disabled
public class Encoder extends LinearOpMode {

    static final double MOTOR_TICK_COUNTS = 537.6;

    MyHardware robot = new MyHardware();
    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    double power = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
       /* robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(power);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(power);

        sleep(1800);*/



        //Go forward
        runtime.startTime();
        encoderDrive(100);
        double time = runtime.time();
       /* sleep(1000);
        robot.leftMotorFront.setPower(0.2);
        robot.rightMotorBack.setPower(0.2);
        robot.leftMotorBack.setPower(0.2);
        robot.rightMotorFront.setPower(0.2);
        encoderDrive(4);
        sleep(500);
*/
        //Spin Carousel Spinner for 5 seconds at a slow power
       // robot.carouselSpinner.setPower(0.5);
        //sleep(5000);

        //Go backwards for 106 inches
       /* encoderDrive(-103);
        sleep(1000);
        robot.leftMotorFront.setPower(0.2);
        robot.rightMotorBack.setPower(0.2);
        robot.leftMotorBack.setPower(0.2);
        robot.rightMotorFront.setPower(0.2);
        encoderDrive(-3);
        sleep(500);
*/
        while (robot.leftMotorFront.isBusy() || robot.rightMotorBack.isBusy() || robot.leftMotorBack.isBusy() || robot.rightMotorFront.isBusy()) {
            //essentially do nothing while you wait for the robot to finish driving to the position
            telemetry.addData("Path", "Driving 18 inches");
            telemetry.update();
            telemetry.addData("time (%f)", time);
            telemetry.update();
        }

        //Stop
        robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    public void encoderDrive(double length) {
        //reset encoders
        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //how many turns do I need for the wheels to go x inches?
        //the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference = 3.14 * 2.95; //pi*diameter
        double rotationsNeeded = length / circumference;
        int encoderDrivingTarget = (int) (rotationsNeeded * MOTOR_TICK_COUNTS);

        //set the target positions
        robot.leftMotorFront.setTargetPosition(encoderDrivingTarget);
        robot.rightMotorBack.setTargetPosition(encoderDrivingTarget);
        robot.leftMotorBack.setTargetPosition(encoderDrivingTarget);
        robot.rightMotorFront.setTargetPosition(encoderDrivingTarget);

        //set the power desired for the motors
        robot.leftMotorFront.setPower(power);
        robot.rightMotorBack.setPower(power);
        robot.leftMotorBack.setPower(power);
        robot.rightMotorFront.setPower(power);

        //set the motors to RUN_TO_POSITION
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

    }


