package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Arm", group="Linear Opmode")
public class Arm {

    private DcMotor arm = null;
    private Servo claw = null;

    private final static double ARM_HOME = 0.2;
    private final static double CLAW_HOME = 0.2;
    private final static double ARM_MIN_RANGE = 0.20;
    private final static double ARM_MAX_RANGE = 0.90;
    private final static double CLAW_MIN_RANGE = 0.20;
    private final static double CLAW_MAX_RANGE = 0.70;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    private void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
    }

}
