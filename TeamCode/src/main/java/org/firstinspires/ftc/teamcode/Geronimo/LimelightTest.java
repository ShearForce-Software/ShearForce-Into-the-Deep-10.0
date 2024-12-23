package org.firstinspires.ftc.teamcode.Geronimo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="LimelightTest")
@Disabled

public class LimelightTest extends LinearOpMode {
        Geronimo control = new Geronimo(true, false, this);
        MecanumDrive_Geronimo drive;
        Pose2d startPose;
        Pose2d deliverToFloorPose;
        Pose2d deliverToBoardPose;
        Pose2d stackPose;

        Action DriveToStack;

        Action GoToSpecimen;
        Action Park;
        Action DriveBackToStack;
        VelConstraint speedUpVelocityConstraint;
        AccelConstraint speedUpAccelerationConstraint;
        VelConstraint slowDownVelocityConstraint;
        AccelConstraint slowDownAccelerationConstraint;
        double stackY = 36;
        double stackX = -59;
        double wallDriveY = 58.5;

        double autoPosition = 3;

        @Override
        public void runOpMode() {
            startPose = new Pose2d(12, 63, Math.toRadians(270));
            // stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

            // Define some custom constraints to use when wanting to go faster than defaults
            speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
            speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
            slowDownVelocityConstraint = new TranslationalVelConstraint(5);
            slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

            /* Initialize the Robot */
            drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
            control.Init(hardwareMap);
            control.InitLimelight(hardwareMap);
            //control.WebcamInit(hardwareMap);
            telemetry.update();
            control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

            while (!isStarted()) {
                telemetry.update();
            }
            resetRuntime();
            control.autoTimeLeft = 0.0;

            // ***************************************************
            // ****  START DRIVING    ****************************
            // ***************************************************



            GoToSpecimen = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(36, 48), Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(57, 48), Math.toRadians(0))
                    .build();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y - control.GetStrafeOffsetInInches("block")), Math.toRadians(180))
                            .build());




            /* Drive to the Board */
            /*
            Actions.runBlocking(
                    new SequentialAction(
                            GoToSpecimen

                            )



            );

             */


            drive.updatePoseEstimate();

            // Build up the Stack to Submerssible Trajectory


            // drive.updatePoseEstimate();
            // Build up the Stack to Wall Trajectory




        /*
        Park = drive.actionBuilder(drive.pose)
                .lineToX(45, slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(48, 56), Math.toRadians(270))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        Park
                )
        );

         */

            control.autoTimeLeft = 30 - getRuntime();
            telemetry.addData("Time left", control.autoTimeLeft);
            telemetry.update();

        }

       /* public Action grabsample() {
            return new org.firstinspires.ftc.teamcode.Geronimo.GeronimoAutoRedLeft.GrabSample();
        }

        public class GrabSample implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }
                packet.put("lock purple pixel", 0);
                return false;  // returning true means not done, and will be called again.  False means action is completely done
            }
            */

        }



