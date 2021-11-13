package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MyTeleop", group="Linear Opmode")
//@Disabled
public class TeleopTest extends LinearOpMode {

    // Declare OpMode members.
    MyHardware robot = new MyHardware();
    private ElapsedTime runtime = new ElapsedTime();
   // double clawPosition = robot.CLAW_HOME;
   // final double CLAW_SPEED = 0.03;

    @Override
    public void runOpMode() {
        double left;
        double right;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            robot.rightMotorFront.setPower(y-x-rx);
            robot.leftMotorBack.setPower(y-x+rx);
            robot.rightMotorBack.setPower(y+x-rx);
            robot.leftMotorFront.setPower(y+x+rx);


            //Assign arm to left joystick of gamepad 1
          //  left = -gamepad1.left_stick_y;

            //Assign Carousel spinner to right joystick of gamepad 1
            //right = -gamepad1.right_stick_y;

            //Assign claw to open if a is pressed, and to close if y is pressed
           /* if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;*/

            //Set position and define maximum and minimum ranges
         //   robot.arm.setPower(left * 0.5);
           // robot.carouselSpinner.setPower(right);
           // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            //robot.claw.setPosition(clawPosition);

            //Update telemetry

          //  telemetry.addData("claw", "%.2f", clawPosition);
          //  telemetry.addData("arm", "%.2f", left * 0.5);
           // telemetry.addData("Carousel Spinner", "%.2f", right);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%d), left back (%d)", robot.leftMotorFront.getCurrentPosition(),
                    robot.leftMotorBack.getCurrentPosition());
            telemetry.addData("Motors", "right front (%d), right back (%d)", robot.rightMotorFront.getCurrentPosition(),
                    robot.rightMotorBack.getCurrentPosition());

            telemetry.update();
            sleep(40);
        }
        }
    }



