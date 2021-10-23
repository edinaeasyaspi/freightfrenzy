
/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class Red_Auto_2 extends LinearOpMode {

    // Declare OpMode members.
    NotTeleopBasicOpMode_Linear robot = new NotTeleopBasicOpMode_Linear();
    double clawPosition = robot.CLAW_HOME;
    final double CLAW_SPEED = 0.01;
    final double ARM_SPEED = 0.5;


    @Override
    public void runOpMode() {

        double left;
        double right;

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.leftMotorBack.setPower(left);
            robot.leftMotorFront.setPower(right);
            robot.rightMotorBack.setPower(left);
            robot.rightMotorFront.setPower(right);


            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);
        }


    }
}
*/
