package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.flywheel.BinarySearch;
@TeleOp(name = "Flywheel Tuning")
public class FlywheelTune extends LinearOpMode {

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

    public double getDistance() {
        final int FIELD_WIDTH = 48500;
        final int FIELD_HEIGHT = 48500;

        DcMotor leftOdo = hardwareMap.get(DcMotor.class, "leftOdo"); // x
        DcMotor rightOdo = hardwareMap.get(DcMotor.class, "rightOdo"); // y
        int leftTicks = leftOdo.getCurrentPosition();
        int rightTicks = rightOdo.getCurrentPosition();

        double offsetX = FIELD_WIDTH - leftTicks; // since we're testing using the red goal
        double offsetY = FIELD_HEIGHT - rightTicks;
        return Math.hypot(offsetX, offsetY);
    }


    double offsetX = (FIELD_WIDTH - 2000) - leftTicks;
    double offsetY = (FIELD_HEIGHT - 2000) - rightTicks;

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

        String tuneMode = "";

        BinarySearch start;
        BinarySearch low;
        BinarySearch high;

        while (opModeIsActive()) {
            if (tuneMode == "first goal") {
                if (gamepad2.dpad_up) {
                    telemetry.addLine("Launched too far at power " + start.getMid());
                    start.goLow();
                    flywheel.setPower(start.getMid());
                }
                if (gamepad2.dpad_down) {
                    telemetry.addLine("Launched too close at power " + start.getMid());
                    start.goHigh();
                    flywheel.setPower(start.getMid());
                }
                if (gamepad2.dpad_right) {
                    telemetry.addLine("Scored at power " + start.getMid());
                    tuneMode = "find lower bound";
                    low = new BinarySearch(start.getLow(), start.getMid());
                    flywheel.setPower(low.getMid());
                    telemetry.addLine("Finding lower bound");
                }
                if (gamepad2.x) {
                    tuneMode = "";
                    start = null;
                    low = null;
                    high = null;
                    flywheel.setPower(0);
                }
            } else if (tuneMode == "find lower bound") {
                if (gamepad2.dpad_down) {
                    telemetry.addLine("Launched too close at power " + low.getMid());
                    low.goHigh();
                    flywheel.setPower(low.getMid());
                }
                if (gamepad2.dpad_right) {
                    telemetry.addLine("Scored at power " + low.getMid());
                    low.goLow();
                    flywheel.setPower(low.getMid());
                }
                if (gamepad2.a) {
                    low = new BinarySearch(start.getLow(), start.getMid());
                    flywheel.setPower(low.getMid());
                    telemetry.addLine("Finding lower bound");
                }
                if (gamepad2.y) {
                    tuneMode = "find upper bound";
                    high = new BinarySearch(start.getMid(), start.getHigh());
                    flywheel.setPower(high.getMid());
                    telemetry.addLine("Finding upper bound");
                }
                if (gamepad2.b) {
                    if (high == null) {
                        telemetry.addLine("Cannot produce results: test upper bound first");
                    } else {
                        telemetry.addLine("The best power at distance " + getDistance() + " is " + (low.getHigh() + high.getLow()) / 2);
                    }
                }
                if (gamepad2.x) {
                    if (high != null) {
                        telemetry.addLine("The best power at distance " + getDistance() + " is " + (low.getHigh() + high.getLow()) / 2);
                    }
                    tuneMode = "";
                    start = null;
                    low = null;
                    high = null;
                    flywheel.setPower(0);
                    telemetry.addLine("Ending test");
                }
            } else if (tuneMode == "find upper bound") {
                if (gamepad2.dpad_up) {
                    telemetry.addLine("Launched too far at power " + high.getMid());
                    high.goLow();
                    flywheel.setPower(high.getMid());
                }
                if (gamepad2.dpad_right) {
                    telemetry.addLine("Scored at power " + high.getMid());
                    high.goHigh();
                    flywheel.setPower(high.getMid());
                }
                if (gamepad2.a) {
                    tuneMode = "find lower bound";
                    low = new BinarySearch(start.getLow(), start.getMid());
                    flywheel.setPower(low.getMid());
                    telemetry.addLine("Finding lower bound");
                }
                if (gamepad2.y) {
                    high = new BinarySearch(start.getMid(), start.getHigh());
                    flywheel.setPower(high.getMid());
                    telemetry.addLine("Finding upper bound");
                }
                if (gamepad2.b) {
                    telemetry.addLine("The best power at distance " + getDistance() + " is " + (low.getHigh() + high.getLow()) / 2);
                }
                if (gamepad2.x) {
                    telemetry.addLine("The best power at distance " + getDistance() + " is " + (low.getHigh() + high.getLow()) / 2);
                    tuneMode = "";
                    start = null;
                    low = null;
                    high = null;
                    flywheel.setPower(0);
                    telemetry.addLine("Ending test");
                }
            } else {
                double y = gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                leftFront.setPower(y + x + rx);
                leftRear.setPower(y - x + rx);
                rightFront.setPower(y + x + rx);
                rightRear.setPower(y - x + rx);

                telemetry.addData("LF", "%.2f");
                telemetry.addData("LR", "%.2f");
                telemetry.addData("RF", "%.2f");
                telemetry.addData("RR", "%.2f");

                if (gamepad2.x) {
                    telemetry.addLine("Starting test at distance " + getDistance());
                    tuneMode = "first goal";
                    start = new BinarySearch(0, 1);
                }
            }
            telemetry.addData("Flywheel", "%.2f", flywheel.getPower());
            telemetry.update();
        }
    }
}