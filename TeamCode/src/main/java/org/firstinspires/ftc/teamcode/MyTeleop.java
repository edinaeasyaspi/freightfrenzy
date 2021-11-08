package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MyTeleop", group="Linear Opmode")
@Disabled
public class MyTeleop extends LinearOpMode {

    // Declare OpMode members.
    PiHardware robot = new PiHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double clawPosition = robot.CLAW_HOME;
    final double CLAW_SPEED = 0.01;

    @Override
    public void runOpMode() {
        double left;
        double leftPower;
        double rightPower;
        double leftPower2;
        double rightPower2;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftPower  = gamepad2.left_stick_y ;
            rightPower = gamepad2.right_stick_y ;

            leftPower2 = gamepad2.left_stick_x;
            rightPower2 = gamepad2.right_stick_x;
            left = -gamepad1.left_stick_y;

            //Forward and backward
            robot.leftMotorBack.setPower(leftPower2);
            robot.rightMotorBack.setPower(-rightPower2);
            robot.leftMotorFront.setPower(-leftPower2);
            robot.rightMotorFront.setPower(rightPower2);

            //Sidewalk
            robot.leftMotorBack.setPower(-leftPower);
            robot.rightMotorBack.setPower(rightPower);
            robot.leftMotorFront.setPower(leftPower);
            robot.rightMotorFront.setPower(-rightPower);


            if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;

            robot.arm.setPower(left);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            telemetry.update();
            sleep(40);
        }
    }
}

