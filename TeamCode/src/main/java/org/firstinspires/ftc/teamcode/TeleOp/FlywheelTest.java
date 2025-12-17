package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.flywheel.Matrix;

@TeleOp(name = "Flywheel Testing")
public class FlywheelTest extends LinearOpMode {

    // ------------------------------------------------------------ //
    //            !! PLEASE ENTER DATA BEFORE RUNNING !!            //
    // Input distance-power pairs obtained through flywheel tuning. //
    // ------------------------------------------------------------ //

    double[] x = {}; // distance
    double[] p = {}; // power

    Function<Double, Double> getPowerFunction(double[] x, double[] p) {
        double[][] a = new double[x.length][2];
        for (int i = 0; i < x.length; i++) {
            a[i][0] = 1 / x[i];
            a[i][1] = 1 / (x[i] * x[i]);
        }
        Matrix A = new Matrix(a);

        double[][] b = new double[p.length][1];
        for (int i = 0; i < p.length; i++) {
            b[i][0] = 1 / (p[i] * p[i]);
        }
        Matrix B = new Matrix(b);

        Matrix solution = A.t().times(A).inverse().times(A.t()).times(B);
        double c = solution.get(0, 0);
        double d = solution.get(1, 0);
        return distance -> Math.sqrt(1 / (c / distance + d / (distance * distance)));
    }

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

        // distance-power function
        Function<Double, Double> distanceToPower = getPowerFunction(d, p);

        telemetry.addLine("Chassis + Flywheel + servo ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
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

            flywheel.setPower(distanceToPower.apply(getDistance()));
            telemetry.addData("Flywheel", "%.2f", flywheel.getPower());
            telemetry.update();
        }
    }
}