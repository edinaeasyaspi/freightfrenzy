package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*@TeleOp(name="MyTeleop", group="Linear Opmode")
@Disabled
public class MyTeleop extends LinearOpMode {

    // Declare OpMode members.
    MyHardware robot = new MyHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double clawPosition = robot.CLAW_HOME;
    final double CLAW_SPEED = 0.03;
    MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        double left;
        double right;

        robot.init(hardwareMap);
        drive = new MecanumDrive(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.setVelocity(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_y, gamepad2.left_stick_button);
            drive.update();
            /*
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            rightPower = gamepad2.right_stick_x;
            leftPower = gamepad2.left_stick_x;
            telemetry.addData("Bumper", "Run Time: " + leftBumper);
            if (rightBumper) {
                //Going Forward and Backward
                robot.leftMotorBack.setPower(leftPower);
                robot.rightMotorBack.setPower(-rightPower);
                robot.leftMotorFront.setPower(-leftPower);
                robot.rightMotorFront.setPower(rightPower);
            }
            else if (leftBumper) {
                //Side strafing
                robot.leftMotorBack.setPower(leftPower);
                robot.rightMotorBack.setPower(rightPower);
                robot.leftMotorFront.setPower(leftPower);
                robot.rightMotorFront.setPower(rightPower);
            }

             */
/*
            //Assign arm to left joystick of gamepad 1
            left = -gamepad1.left_stick_y;

            //Assign Carousel spinner to right joystick of gamepad 1
            right = -gamepad1.right_stick_y;

            //Assign claw to open if a is pressed, and to close if y is pressed
            if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;

            //Set position and define maximum and minimum ranges
            robot.arm.setPower(left*0.5);
            robot.carouselSpinner.setPower(right);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            //Update telemetry

            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.addData("arm", "%.2f", left*0.5);
            telemetry.addData("Carousel Spinner","%.2f",right);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%d), left back (%d)", robot.leftMotorFront.getCurrentPosition(),
                    robot.leftMotorBack.getCurrentPosition());
            telemetry.addData("Motors", "right front (%d), right back (%d)", robot.rightMotorFront.getCurrentPosition(),
                    robot.rightMotorBack.getCurrentPosition());

            drive.displayTelemetry(telemetry);
            telemetry.update();
            sleep(40);
        }
    }
    /**
     * Scale a joystick value to smooth it for motor setting.
     * The cube results in finer control at slow speeds.
     */
   /* public static double ScaleMotorCube(double joyStickPosition) {
        return (double) Math.pow(joyStickPosition, 3.0);
    }
}*/

