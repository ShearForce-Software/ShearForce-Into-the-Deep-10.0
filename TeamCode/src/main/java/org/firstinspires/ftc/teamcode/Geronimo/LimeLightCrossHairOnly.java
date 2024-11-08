package org.firstinspires.ftc.teamcode.Geronimo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimeLightCrossHairOnly extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    Geronimo control;

    // Constants
    private static final double KpDistance = -0.1; // Proportional control constant for distance adjustment
    private static final double KpAim = 0.1; // Proportional control constant for aiming adjustment

    @Override
    public void runOpMode() {
        // Initialize hardware
        control = new Geronimo(false, false, this);
        control.InitLimelight(hardwareMap);
        control.Init(hardwareMap);
        // Set Limelight to pipeline 1

        telemetry.addLine("Init Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            String TargetImageName = "bottle";

            //Use the AlignToTargetImage to get tx and ty values
            List<Double> offsets = control.AlignToTargetImage(TargetImageName, 0.1, 0.8);
            double distanceAdjust = offsets.get(0);
            double strafeAdjust = offsets.get(1);


//            control.moveRobot(distanceAdjust, strafeAdjust, 0);
//
//            while(distanceAdjust > 1.0){
//                List<Double> offsetsFine = control.AlignToTargetImage("testingobject", 0.1, 0.8);
//                 distanceAdjust = offsets.get(0);
//                 strafeAdjust = offsets.get(1);
//                control.moveRobot(distanceAdjust, strafeAdjust, 0);
//            }
            //Calculate adjustments for driving

            telemetry.addLine("X OffSet: " + distanceAdjust);
            telemetry.addLine("Y OffSet:" + strafeAdjust);


        telemetry.update();
        }

    }
}