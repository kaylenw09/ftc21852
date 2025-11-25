package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "kaylenChassis")
public class KaylenChassis extends LinearOpMode {

    // Chassis motors

    // wheel motor names: fl, bl, fr, br
    private static final String LF_NAME = "fl";
    private static final String LR_NAME = "bl";
    private static final String RF_NAME = "fr";
    private static final String RR_NAME = "br";
    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String SERVO_NAME = "servo";


    // find motor class
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotor flywheel;
    private Servo servo;



    // Gate motor
    private boolean gateTriggerWasPressed = false;


    @Override
    public void runOpMode() {
        // Initialize chassis motors
        leftFront = hardwareMap.get(DcMotorEx.class, LF_NAME);
        leftRear  = hardwareMap.get(DcMotorEx.class, LR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RF_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, RR_NAME);
        flywheel = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        servo = hardwareMap.get(Servo.class, SERVO_NAME);


        // Motor directions
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);


        // Zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry.addLine("Chassis + Flywheel + servo ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double y  = gamepad1.left_stick_y ;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x ;
            //chassis
            leftFront.setPower(y + x + rx);
            leftRear.setPower(y - x + rx);
            rightFront.setPower(y + x + rx);
            rightRear.setPower(y - x + rx);

            telemetry.addData("LF", "%.2f");
            telemetry.addData("LR", "%.2f");
            telemetry.addData("RF", "%.2f");
            telemetry.addData("RR", "%.2f");
            telemetry.addData("Flywheel", "%.2f", flywheel.getPower());
            telemetry.update();
        }
    }
}
