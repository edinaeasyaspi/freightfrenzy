package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Teleoptest", group="Linear Opmode")
//@Disabled
public class TeleopBasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    NotTeleopBasicOpMode_Linear robot = new NotTeleopBasicOpMode_Linear();
    double clawPosition = robot.CLAW_HOME;
    final double CLAW_SPEED = 0.01;


    @Override
    public void runOpMode() {

        double left;
        //double right;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                    clawPosition -= CLAW_SPEED;

            left = -gamepad1.left_stick_y;
            //right = -gamepad1.right_stick_y;
            robot.arm.setPower(left);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.addData("left", "%.2f", left);
            //telemetry.addData("right", "%.2f", right);
            telemetry.update();

            sleep(40);



        }


        }
    }

