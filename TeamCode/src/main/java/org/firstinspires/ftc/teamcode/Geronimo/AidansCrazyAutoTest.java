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
//import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Aidan'sCrazyAutoTest", preselectTeleOp =
        "Geronimo_Manual_Control")
// @Disabled
public class AidansCrazyAutoTest extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;
    Vector2d VectorTwo;
    Action DeliverStartingSpecimen;
    Action DriveToSamplesandDeliver1;
    Action DriveToSamplesandDeliver2;
    Action DriveToSamplesandDeliver3;
    Action DriveToSubmersible1;
    Action DriveToSubmersible2;
    Action DriveToSubmersible3;
    Action grabSpecimenfromwall;
    Action ParkinDeck;
    float currentXPosition;
    float currentYPosition;
    double currentHeading;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;


    double autoPosition = 3;

    public void runOpMode(){
        startPose = new Pose2d(12,-64, Math.toRadians(90));
        VectorTwo = new Vector2d(5, -30);
        // stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.AutoStartPosition();
        //telemetry.addData("Current X position", );

        //control.WebcamInit(hardwareMap);
        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        while(!isStarted()){
            telemetry.update();
        }
        resetRuntime();
        control.autoTimeLeft = 0.0;
        control.SetClawPosition(Geronimo.CLAW_MAX_POS);

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************

        DeliverStartingSpecimen = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(8,-35), Math.toRadians(90))
                .build();
        DriveToSamplesandDeliver1 = drive.actionBuilder(new Pose2d(12, -35, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(25,-48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-24), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36,-12), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(44,-12),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(44,-54), Math.toRadians(270),slowDownVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(47,-60), Math.toRadians(270))
                .build();
        DriveToSamplesandDeliver2 = drive.actionBuilder(new Pose2d(44,-54, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(44, -17), Math.toRadians(270), slowDownVelocityConstraint)
                .splineToConstantHeading(new Vector2d(54,-17), Math.toRadians(270))
                .lineToYConstantHeading(-54,slowDownVelocityConstraint)
               // .strafeToLinearHeading(new Vector2d(44, -54), Math.toRadians(270))
                .build();
        DriveToSamplesandDeliver3 = drive.actionBuilder(new Pose2d(54,-54,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(54, -12), Math.toRadians(270), slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(62,-12), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(62,-54), Math.toRadians(270), slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(50,-54),Math.toRadians(270))
                .build();
      /*  DriveToSubmersible1 = drive.actionBuilder(new Pose2d(62,-54, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(48, -54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(7,-35), Math.toRadians(90))
                .build();
        DriveToSubmersible2 = drive.actionBuilder(new Pose2d(7,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(7,-35), Math.toRadians(90)) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible3 = drive.actionBuilder(new Pose2d(7,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(270))
                 .strafeToLinearHeading(new Vector2d(7,-35), Math.toRadians(90))
                .build();

       */
         ParkinDeck = drive.actionBuilder(new Pose2d(50,-54,Math.toRadians(270)))
                 .splineTo(new Vector2d(49,-45),Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(49,-58), Math.toRadians(90))
                 .turnTo(Math.toRadians(90))
                        .build();
/*

        BoxTraj = drive.actionBuilder(new Pose2d(-12,-30,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, -36), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                .build();

 */

        /* Drive to the Board */
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DeliverStartingSpecimen,
                                deliverLowSpecimen()),
                        releaseSpecimen(),
                        new SleepAction(.3),
                        DriveToSamplesandDeliver1,
                        DriveToSamplesandDeliver2,
                        DriveToSamplesandDeliver3,
                        ParkinDeck,
                        stowPosition(),
                        new SleepAction(5))






        );


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

        control.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", control.autoTimeLeft);
        telemetry.update();

    }

    public Action grabSpecimenfromwall (){return new GrabSpecimenFromWall();}
    public class GrabSpecimenFromWall implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.limelightHasTarget();
                control.SpecimenPickupFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action deliverLowSpecimen (){return new DeliverLowSpecimen();}
    public class DeliverLowSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverLow();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action releaseSpecimen (){return new ReleaseSpecimen();}
    public class ReleaseSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetClawPosition((Geronimo.CLAW_MIN_POS));
                control.SpecimenPickupFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action stowPosition (){return new StowPosition();}
    public class StowPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWall();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
}

