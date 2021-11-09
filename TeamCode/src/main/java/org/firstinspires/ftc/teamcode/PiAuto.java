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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
Basic autonomous using encoders to stay on course.
 */

@Autonomous(name = "Auto", group = "Auto")
//@Disabled
public class PiAuto extends LinearOpMode {

    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static double COUNTS_PER_MOTOR_REV = 0;
    // Calculated at start of opMode.
    static double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV;
    /* Declare OpMode members. */
    private PiHardware robot = new PiHardware();
    private ElapsedTime runtime = robot.runtime;

    @Override
    public void runOpMode() {
        initMotor();
        COUNTS_PER_MOTOR_REV = robot.leftMotorBack.getMotorType().getTicksPerRev();
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, -3, 3, 4.0);  // S2: Turn left with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S3: Forward 12 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -1, -1, 4.0);  // S3: Reverse 1 Inch with 3 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void initMotor() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotorFront.getCurrentPosition(),
                robot.leftMotorBack.getCurrentPosition(),
                robot.rightMotorFront.getCurrentPosition(),
                robot.rightMotorBack.getCurrentPosition());
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotorFront.setTargetPosition(newLeftTarget);
            robot.leftMotorBack.setTargetPosition(newLeftTarget);
            robot.rightMotorFront.setTargetPosition(newRightTarget);
            robot.rightMotorBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rightMotorFront.setPower(Math.abs(speed));
            robot.leftMotorBack.setPower(Math.abs(speed));
            robot.rightMotorFront.setPower(Math.abs(speed));
            robot.rightMotorBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.leftMotorBack.isBusy() &&
                    robot.rightMotorFront.isBusy() &&
                    robot.leftMotorFront.isBusy() &&
                    robot.rightMotorBack.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition(),
                        robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotorFront.setPower(0);
            robot.leftMotorBack.setPower(0);
            robot.rightMotorFront.setPower(0);
            robot.rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
