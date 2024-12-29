package org.firstinspires.ftc.teamcode.summerChassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

@TeleOp(name = "AlignToTarget", group = "TeleOp")
public class AlignToTarget extends LinearOpMode {
    private SummerChassis control;
    private MecanumDrive_summerChassis drive;

    @Override
    public void runOpMode() {
        // We do this AFTER runOpMode() starts, so hardwareMap is valid.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Make your objects here
        control = new SummerChassis(false, false, this);
        control.Init(hardwareMap);
        control.InitLimelight(hardwareMap);

        // Setup the drive AFTER hardwareMap is valid
        drive = new MecanumDrive_summerChassis(hardwareMap, startPose);

        telemetry.addLine("Ready to check Limelight offset...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check the offset every loop
            double strafeOffset = control.GetStrafeOffsetInInches("block");

            // If user presses up, and the target offset is not zero
            if (gamepad1.dpad_up && Math.abs(strafeOffset) > 0.2) {
                // If strafeOffset is positive, maybe you want to strafe left or right?
                // By default, Road Runner +y is left, -y is right (depending on your config).
                // Let's do the simplest approach: new Vector2d(drive.pose.position.x, strafeOffset)
                // so the robot tries to correct left or right from the origin.

                Action strafeAction = drive.actionBuilder(drive.pose)
                        .strafeTo(
                                new Vector2d(0, 2)
                                // Or maybe drive.pose.heading + 0 or Math.toRadians(0)

                        )
                        .build();

                Actions.runBlocking(strafeAction);
            }

            telemetry.addData("Strafe Offset", strafeOffset);
            telemetry.addLine("Press dpad_up to strafe...");
            telemetry.update();
        }
    }
}
