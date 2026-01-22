package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name = "Camera")
public class Camera extends LinearOpMode {

    private Limelight3A limelight;
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        limelight.start();
        sleep(200);

        while (opModeIsActive()) {
            double distance = getDistance();

            if (distance < 0) {
                telemetry.addLine("No valid AprilTag");
            } else {
                telemetry.addData("Distance (m)", distance);
            }

            telemetry.update();
        }
    }


    public double getDistance() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return -1;
        }

        FiducialResult tag = tags.get(0);

        Pose3D pose = tag.getCameraPoseTargetSpace();
        Position p = pose.getPosition();

        double x = p.x;
        double y = p.y;
        double z = p.z;

        // 原始单位通常是 mm
        double distanceMm = Math.sqrt(x * x + y * y + z * z);

        // 转成 meters
        return distanceMm / 1000.0;
    }
}
