package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

@TeleOp(name = "Geronimo Limelight Offset (All Zeros)", group = "Geronimo")
public class Geronimo_LimeLight_OffsetCheck extends LinearOpMode {

    private Geronimo theRobot;
    private MecanumDrive_Geronimo drive;

    // We’ll store the Limelight strafe offset here
    double[] offsetInches;

    @Override
    public void runOpMode() {

        // 1) Initialize the robot and Limelight
        theRobot = new Geronimo(false, false, this);
        theRobot.Init(hardwareMap);
        theRobot.InitLimelight(hardwareMap);

        // 2) Initialize the Road Runner Mecanum drive at (0,0,0)
        //    We won't track pose or call setPoseEstimate(), so it's always "starting from zero"
        drive = new MecanumDrive_Geronimo(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Initialized (start pos = 0). Press Play to begin.");
        telemetry.update();
        waitForStart();

        // Variables for edge-detection of dpad presses
        boolean lastDpadUp = false;
        boolean lastDpadRight = false;

        while (opModeIsActive()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadRight = gamepad1.dpad_right;

            //----------------------------------------------------
            //  1) DPAD_UP => GET THE LIMELIGHT OFFSET (ONCE)
            //----------------------------------------------------
            if (currentDpadUp && !lastDpadUp) {
                //theRobot.SetSlideRotatorArmToPosition(100);
                theRobot.SetSlideToPosition(1400);
                theRobot.SetIntakeBoxRotatorPosition(0.04);
                theRobot.SetSmallArmHangerPosition(0.365);
sleep(100);
                // Retrieve the offset from the Limelight
                offsetInches = theRobot.GetStrafeOffsetInInches("block");
                // If offsetInches == 0, it likely means "no target found"
                if (Math.abs(offsetInches[0]) < 0.001) {
                    telemetry.addLine("No target detected => offset = 0");

                } else {
                    telemetry.addData("Offset X(inches)", "%.2f", offsetInches[0]);
                    telemetry.addData("Offset Y(inches)", "%.2f", offsetInches[1]);
                }
                telemetry.update();
            }


            //----------------------------------------------------
            //  2) DPAD_RIGHT => STRAFE USING THAT OFFSET
            //----------------------------------------------------
            if (currentDpadRight && !lastDpadRight) {
                // Build a Road Runner action that starts at (0,0,0)
                // and strafes in Y by `offsetInches`.

                Action strafeAction = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToConstantHeading(new Vector2d(-offsetInches[1], offsetInches[0]))
                        .build();

                // Run the strafe action (blocking) to move the robot
                Actions.runBlocking(strafeAction);

                // Telemetry
                telemetry.addData("Strafed", "%.2f inches", offsetInches[0]);
                telemetry.addData("Strafed", "%.2f inches", offsetInches[1]);
                telemetry.update();
            }

            lastDpadUp = currentDpadUp;
            lastDpadRight = currentDpadRight;
        }
    }
}
