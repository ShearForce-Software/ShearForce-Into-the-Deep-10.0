package org.firstinspires.ftc.teamcode.summerChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="Strafe Right 10 Inches", group="SummerChassis")
public class StrafeRightAutoRoutine extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1) Create a starting Pose2d.
        //    For demonstration, let’s assume we start at x=0, y=0, heading=0 (facing "forward").
        Pose2d startPose = new Pose2d(0, 0, 0);

        // 2) Construct your MecanumDrive_summerChassis.
        //    This is the custom Road Runner wrapper you have in your code base.
        MecanumDrive_summerChassis drive = new MecanumDrive_summerChassis(hardwareMap, startPose);

        // 3) Build an Action (trajectory) that strafes the robot to the right by 10 inches.
        //    Because Road Runner’s coordinate system uses +y as “left,” going to (0, -10)
        //    will strafe ~10 inches to the right from the current position.
        Action strafeRightAction = drive.actionBuilder(drive.pose)  // Starting from drive.pose (= startPose)
                .strafeToConstantHeading(new Vector2d(0,6))
                .build();


        // 4) Wait for the user to press PLAY
        telemetry.addLine("Ready to strafe right 10 inches...");
        telemetry.update();
        waitForStart();

        // 5) Once PLAY is pressed, run the strafe action to completion
        if (opModeIsActive()) {
            Actions.runBlocking(strafeRightAction);

            // After the action is complete, the robot should have moved ~10 inches to the right
            // and then automatically stopped.

            telemetry.addLine("Strafe action complete!");
            telemetry.update();
        }
    }
}
