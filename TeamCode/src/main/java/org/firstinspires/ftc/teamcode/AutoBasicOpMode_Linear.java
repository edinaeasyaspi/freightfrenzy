package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Basic: Autotest", group="Linear Opmode")
@Disabled
public class AutoBasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    NotTeleopBasicOpMode_Linear robot = new NotTeleopBasicOpMode_Linear();
    double clawPosition = robot.CLAW_HOME;
    final double ARM_SPEED = 0.8;
    public DcMotor arm = null;
    public final static int ARM_HOME = 0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            arm.setPower(ARM_SPEED);

           if (gamepad1.a)
               sleep(20);
           else if (gamepad1.b)
               sleep(40);
           else if (gamepad1.x)
               sleep(60);
           else if(gamepad1.y)
               arm.setPower(ARM_HOME);

            telemetry.addData("arm", "%.2f", arm);
            telemetry.update();


        }


        }
    }

