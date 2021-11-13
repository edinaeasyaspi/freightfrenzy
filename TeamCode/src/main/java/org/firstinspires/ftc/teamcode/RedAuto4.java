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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Redwh4")
//@Disabled
public class RedAuto4 extends LinearOpMode {

    /* Declare OpMode members. */
    MyHardware robot = new MyHardware();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.1;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    Orientation angles;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        waitForStart();

        gyroDrive(DRIVE_SPEED,5,0);
        gyroTurn(TURN_SPEED,-90);
        gyroHold(TURN_SPEED,-90,0.5);
        gyroDrive(DRIVE_SPEED,4,-90);
        gyroDrive(DRIVE_SPEED, 23, 0.0);    // Drive forward 5 inches

        gyroDrive(DRIVE_SPEED,-18,0);
        gyroTurn(TURN_SPEED,90);
        gyroHold(TURN_SPEED,90,0.5);
        gyroDrive(DRIVE_SPEED,60,90);
    }


    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newLeftTarget2;
        int     newRightTarget2;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotorBack.getCurrentPosition() + moveCounts;
            newRightTarget = robot.leftMotorFront.getCurrentPosition() + moveCounts;
            newLeftTarget2 = robot.rightMotorFront.getCurrentPosition() + moveCounts;
            newRightTarget2 = robot.rightMotorBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotorBack.setTargetPosition(newLeftTarget);
            robot.leftMotorFront.setTargetPosition(newRightTarget);
            robot.rightMotorFront.setTargetPosition(newLeftTarget2);
            robot.rightMotorBack.setTargetPosition(newRightTarget2);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);
            robot.rightMotorBack.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.leftMotorBack.isBusy() && robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy() && robot.rightMotorBack.isBusy())) {

                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotorBack.setPower(leftSpeed);
                robot.leftMotorFront.setPower(leftSpeed);
                robot.rightMotorFront.setPower(rightSpeed);
                robot.rightMotorBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotorBack.getCurrentPosition(),
                                                             robot.rightMotorBack.getCurrentPosition());
                telemetry.addData("gyro/Err/St", "%5.2f/%5.2f/%5.2f", angles.firstAngle,error, steer);
                telemetry.addData("leftSpeed:rightspeed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);
            robot.rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();

        }
        // Stop all motion;
        robot.leftMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotorFront.setPower(leftSpeed);
        robot.leftMotorBack.setPower(leftSpeed);
        robot.rightMotorBack.setPower(rightSpeed);
        robot.rightMotorFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("gyro/Err/St", "%5.2f/%5.2f/%5.2f", angles.firstAngle,error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);


        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
