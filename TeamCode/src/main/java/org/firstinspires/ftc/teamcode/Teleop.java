package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop", group="Linear Opmode")
@Disabled
public class Teleop extends LinearOpMode {

    // Create object of MyHardware to be able to use
    MyHardware robot = new MyHardware();

    @Override
    public void runOpMode() {
        //Declare variables
        double clawPosition = robot.CLAW_HOME;
        final double CLAW_SPEED = 0.03;
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Mecanum Drive
            //Get power from gamepad2
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x*1.1;
            double rx = gamepad2.right_stick_x;

            robot.rightMotorFront.setPower(y-x-rx);
            robot.leftMotorBack.setPower(y-x+rx);
            robot.rightMotorBack.setPower(y+x-rx);
            robot.leftMotorFront.setPower(y+x+rx);

            //Assign carousel spinner to right joystick of gamepad1
            robot.carouselSpinner.setPower(-gamepad1.right_stick_y);

            //Assign arm to left joystick of gamepad1
            robot.arm.setPower(-gamepad1.left_stick_y );

            //Assign claw to open if a is pressed, and to close if y is pressed
            if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;

            //Set position and define maximum and minimum ranges
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            //Update telemetry
            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.addData("arm", "%.2f", -gamepad1.left_stick_y * 0.5);
            telemetry.addData("Carousel Spinner", "%.2f", -gamepad1.right_stick_y);
            telemetry.addData("Motors", "left front (%d), left back (%d)", robot.leftMotorFront.getCurrentPosition(),
                    robot.leftMotorBack.getCurrentPosition());
            telemetry.addData("Motors", "right front (%d), right back (%d)", robot.rightMotorFront.getCurrentPosition(),
                    robot.rightMotorBack.getCurrentPosition());

            telemetry.update();
            sleep(40);
        }
        }
    }



