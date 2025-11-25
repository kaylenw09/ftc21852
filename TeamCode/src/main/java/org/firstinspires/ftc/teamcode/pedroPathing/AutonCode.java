package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutonCode")
public class AutonCode extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;

    private DcMotor forwardOdo, sidewaysOdo;


    private IMU pinPoint;

    private DcMotor flywheel, gateMotor;

    private static final double WHEEL_DIAMETER_METERS = 0.048;    // 48 mm
    private static final double TICKS_PER_REV = 8192.0;          // REV Through-Bore encoder
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_METERS;
    private static final double TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;


    private static final double HEADING_KP = 0.04;


    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    private double lastForwardTicks = 0.0;
    private double lastSideTicks = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {


        lf = hardwareMap.get(DcMotor.class, "fl");
        rf = hardwareMap.get(DcMotor.class, "fr");
        lb = hardwareMap.get(DcMotor.class, "bl");
        rb = hardwareMap.get(DcMotor.class, "br");


        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);


        forwardOdo = hardwareMap.get(DcMotor.class, "Vodo");  // vertical (forward/back)
        sidewaysOdo = hardwareMap.get(DcMotor.class, "Hodo"); // horizontal (left/right)


        forwardOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sidewaysOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sidewaysOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lastForwardTicks = forwardOdo.getCurrentPosition();
        lastSideTicks = sidewaysOdo.getCurrentPosition();


        pinPoint = hardwareMap.get(IMU.class, "pinPoint");
        // Replace these with how your REV hub is mounted. If you aren't sure, try the common mounting.
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDir, usbDir);
        IMU.Parameters params = new IMU.Parameters(orientationOnRobot);
        pinPoint.initialize(params);


        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        telemetry.addData("Status", "Init complete");
        telemetry.update();

        waitForStart();


        moveForwardMeters(0.5, 0.3);
        rotateToAngle(-45.0, 0.25);
        moveRightMeters(0.5, 0.3);



        flywheel.setPower(0);
        stopMotors();
    }


    private void updateOdometry() {
        double currentForwardTicks = forwardOdo.getCurrentPosition();
        double currentSideTicks = sidewaysOdo.getCurrentPosition();


        double deltaForwardTicks = currentForwardTicks - lastForwardTicks;
        double deltaSideTicks = currentSideTicks - lastSideTicks;


        double deltaForward = deltaForwardTicks / TICKS_PER_METER;
        double deltaSide = deltaSideTicks / TICKS_PER_METER;


        lastForwardTicks = currentForwardTicks;
        lastSideTicks = currentSideTicks;


        robotHeading = getHeading();
        double headingRad = Math.toRadians(robotHeading);


        double deltaX =  deltaForward * Math.cos(headingRad) - deltaSide * Math.sin(headingRad);
        double deltaY =  deltaForward * Math.sin(headingRad) + deltaSide * Math.cos(headingRad);

        robotX += deltaX;
        robotY += deltaY;
    }


    private void moveForwardMeters(double meters, double power) {
        double direction = Math.signum(meters);
        double targetMeters = Math.abs(meters);

        double startForwardTicks = forwardOdo.getCurrentPosition();
        double startHeading = getHeading();

        long moveStart = System.currentTimeMillis();

        while (opModeIsActive()) {
            updateOdometry();

            double traveled = Math.abs((forwardOdo.getCurrentPosition() - startForwardTicks) / TICKS_PER_METER);
            if (traveled >= targetMeters) break;

            if(System.currentTimeMillis() - moveStart > 5000) {
                telemetry.addLine("Move Forward Timeout");
                telemetry.update();
                break;
            }

            double headingError = normalizeAngle(getHeading() - startHeading);
            double correction = -HEADING_KP * headingError;

            double leftPower  = direction * power + correction;
            double rightPower = direction * power - correction;

            lf.setPower(leftPower);
            lb.setPower(leftPower);
            rf.setPower(rightPower);
            rb.setPower(rightPower);

            telemetry.addData("Mode", "Forward");
            telemetry.addData("Traveled(m)", "%.3f", traveled);
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }

        stopMotors();
    }



    private void moveRightMeters(double meters, double power) {
        double direction = Math.signum(meters);
        double targetMeters = Math.abs(meters);


        double startSideTicks = sidewaysOdo.getCurrentPosition();

        while (opModeIsActive()) {
            updateOdometry();

            double traveled = Math.abs((sidewaysOdo.getCurrentPosition() - startSideTicks) / TICKS_PER_METER);
            if (traveled >= targetMeters) break;

            double headingError = normalizeAngle(getHeading() - robotHeading);
            double correction = -HEADING_KP * headingError;


            double strafe = direction * power;
            double lfPower =  strafe + correction;
            double rfPower = -strafe - correction;
            double lbPower = -strafe + correction;
            double rbPower =  strafe - correction;

            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);

            telemetry.addData("Mode", "Strafe");
            telemetry.addData("target(m)", targetMeters);
            telemetry.addData("traveled(m)", "%.3f", traveled);
            telemetry.addData("X", "%.3f", robotX);
            telemetry.addData("Y", "%.3f", robotY);
            telemetry.addData("Heading", "%.2f", robotHeading);
            telemetry.update();
        }

        stopMotors();
    }


    private void rotateToAngle(double targetAngle, double maxPower) {


        long startTime = System.currentTimeMillis();
        long timeout = 4000;

        while (opModeIsActive()) {

            double current = getHeading();
            double error = normalizeAngle(targetAngle - current);  // correct sign


            if (Math.abs(error) < 1.0) break;


            if (System.currentTimeMillis() - startTime > timeout) {
                telemetry.addLine("TIMEOUT - STOP ROTATION");
                telemetry.update();
                break;
            }


            double power = Math.abs(error) * 0.01;
            power = Math.max(0.06, Math.min(power, maxPower));

            double turn = Math.signum(error) * power;


            lf.setPower(turn);
            lb.setPower(turn);
            rf.setPower(-turn);
            rb.setPower(-turn);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", current);
            telemetry.addData("Error", error);
            telemetry.addData("Power", turn);
            telemetry.update();
        }

        stopMotors();
    }


    private double getHeading() {

        return pinPoint.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    private double normalizeAngle(double angle) {
        angle = ((angle % 360) + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private void stopMotors() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }
}
