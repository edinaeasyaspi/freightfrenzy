package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/*
Simple linear opMode using mecanum drive.
 */

@TeleOp(name = "TeleOp", group = "TeleOp")
//@Disabled
public class PiTeleop extends LinearOpMode {

    final double CLAW_SPEED = 0.01;
    final double ARM_SPEED = .2;
    // Declare OpMode members.
    PiHardware robot = new PiHardware();
    double clawPosition = robot.CLAW_HOME;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(robot.hwMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.setVelocity(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button);
            drive.update();

            if (gamepad1.a)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.y)
                clawPosition -= CLAW_SPEED;

            robot.arm.setPower(ARM_SPEED);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.addData("Arm", "%d", robot.arm.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //sleep(40);
        }
    }
}

