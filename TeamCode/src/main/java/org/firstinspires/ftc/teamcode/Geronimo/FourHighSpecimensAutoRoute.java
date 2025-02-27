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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Four High Specimens Auto Route", preselectTeleOp =
        "Geronimo 1 Manual Control")
// @Disabled
public class FourHighSpecimensAutoRoute extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;

    // Trajectories
    Action DeliverStartingSpecimen;
    Action DriveToSamplesandDeliver1;
    Action DriveToSamplesandDeliver2;
    Action DriveToSamplesandDeliver3;
    Action DrivetoDeck1;
    Action DriveToSubmersible1;
    Action DrivetoDeck2;
    Action DriveToSubmersible2;
    Action DrivetoDeck3;
    Action DriveToSubmersible3;
    Action ParkinDeck;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint normalVelocityConstraint;
    AccelConstraint normalAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    VelConstraint intakeVelocityConstraint;
    VelConstraint humanPlayerVelocityConstraint;

    public void runOpMode(){
        startPose = new Pose2d(9,-64, Math.toRadians(270));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(70.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-50.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);


        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        //  -- need to adjust this starting position to keep the specimen out of the wall Check
        control.AutoStartPosition();
        control.SetSwiperPosition(Geronimo.SWIPER_MAX_POS);
        control.SetSwiper2Position(Geronimo.SWIPER2_MIN_POS);
        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        while(!isStarted()){
            telemetry.update();
        }
        resetRuntime();
        Geronimo.autoTimeLeft = 0.0;
        control.SetClawPosition(Geronimo.CLAW_MAX_POS);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DeliverStartingSpecimen = drive.actionBuilder(startPose)
                //.strafeToLinearHeading(new Vector2d(4,-30), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(0,-30), Math.toRadians(270), slowDownVelocityConstraint)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(0,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSamplesandDeliver1 = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(270)))
                //.strafeToLinearHeading(new Vector2d(0,-48), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(36,-48,Math.toRadians(270)), Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                //.splineToConstantHeading(new Vector2d(36,-48), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(36,-48,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(38, -40),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38,-15), Math.toRadians(270), slowDownVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(48,-15),Math.toRadians(270),normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(48,-15), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToConstantHeading(new Vector2d(48,-54), normalVelocityConstraint, normalAccelerationConstraint)
                .build();
// Fix
        DriveToSamplesandDeliver2 = drive.actionBuilder(new Pose2d(48,-54, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(48, -16.5), Math.toRadians(270), normalVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(56,-16.5),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(56,-16.5), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToConstantHeading(new Vector2d(56,-54), normalVelocityConstraint, normalAccelerationConstraint)
                .build();

        /*DriveToSamplesandDeliver3 = drive.actionBuilder(new Pose2d(56,-54,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(54, -12), Math.toRadians(270), slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(60.75,-12), Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(63,-17), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60.75,-54), Math.toRadians(270))
                //.lineToYConstantHeading(-54)
                //.strafeToLinearHeading(new Vector2d(48,-54),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(44,-46),Math.toRadians(270))
                .build();
*/
        DrivetoDeck1 = drive.actionBuilder(new Pose2d(56,-54,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(38,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38, -63),Math.toRadians(270), humanPlayerVelocityConstraint)
                .build();

        DriveToSubmersible1 = drive.actionBuilder(new Pose2d(38,-63, Math.toRadians(270)))
                .setReversed(true)
                //.strafeToLinearHeading(new Vector2d(48, -54), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(2, -54), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(2,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(2,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();
        DrivetoDeck2 = drive.actionBuilder(new Pose2d(2,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(38,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38,-63), Math.toRadians(270), humanPlayerVelocityConstraint) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible2 = drive.actionBuilder(new Pose2d(38,-63,Math.toRadians(270)))
                .setReversed(true)
                //.strafeToLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(4,-39,Math.toRadians(270)), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(4,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(4,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();
        DrivetoDeck3 = drive.actionBuilder(new Pose2d(4,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(38,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38,-63), Math.toRadians(270), humanPlayerVelocityConstraint) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible3 = drive.actionBuilder(new Pose2d(38,-63,Math.toRadians(270)))
                .setReversed(true)
                //.strafeToLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(3,-39,Math.toRadians(270)), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(5,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(5,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();


         ParkinDeck = drive.actionBuilder(new Pose2d(5,-30,Math.toRadians(270)))
                 //Pose 2D 50,-54, 270
                 //.splineTo(new Vector2d(49,-45),Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(6,-35), Math.toRadians(270), intakeVelocityConstraint)
                 //.strafeToLinearHeading(new Vector2d(36,-58), Math.toRadians(90), intakeVelocityConstraint)
                 //.splineToLinearHeading(new Pose2d(30,-48,Math.toRadians(90)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                 .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(90)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                 //.strafeToLinearHeading(new Vector2d(2,-50), Math.toRadians(270))
                 //.strafeToLinearHeading(new Vector2d(49,-58), Math.toRadians(90), speedUpVelocityConstraint)
                 // .turnTo(Math.toRadians(90))
                 .build();
        //before:
        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        // deliver pre-loaded specimen
                        new ParallelAction(DeliverStartingSpecimen
                                , new SequentialAction(
                                        new SleepAction(0.3),
                                deliverSpecimenHigh())),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.6),
                        releaseSpecimen(),
                        new SleepAction(.3),

                        // Gather the 3 floor samples into the observation area
                        new ParallelAction(DriveToSamplesandDeliver1
                                , new SequentialAction(new SleepAction(.3),
                                        //don't call stow; call wall position
                                        slidestozero(), rotatorarmstozero(), stowPosition()
                        )),
                        DriveToSamplesandDeliver2,
                        // 1st Initial Delivery
                        // Drive to the wall and prepare to grab a specimen
                        new ParallelAction(DrivetoDeck1,
                                grabSpecimenfromwall()),

                        // TODO -- need to test how small we can make these sleep actions, these servos are pretty fast this year
                        // grab the specimen off of the wall
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),

                        // Deliver the specimen to the High bar
                        new ParallelAction(DriveToSubmersible1
                                , deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.6),
                        releaseSpecimen(),
                        new SleepAction(.3),
                        //2nd delivery
                        new ParallelAction(DrivetoDeck2,
                                new SequentialAction(//don't call stow; call wall position
                                slidestozero(), rotatorarmstozero(), stowPosition(), grabSpecimenfromwall())),
                        grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall(),
                        new SleepAction(.5),
                        new ParallelAction(DriveToSubmersible2
                                , deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.6),
                        releaseSpecimen(),
                        new SleepAction(.3),
                        //3rd delivery
                        new ParallelAction(DrivetoDeck3,
                                new SequentialAction(//don't call stow; call wall position
                                        slidestozero(), rotatorarmstozero(), stowPosition(), grabSpecimenfromwall()))
                        ,grabSpecimen(),
                        new SleepAction(.3),
                        liftSpecimenoffWall()
                        /*
                        new SleepAction(.5),
                        new ParallelAction(DriveToSubmersible3
                                , deliverSpecimenHigh()),
                        finishdeliverSpecimenHigh(),
                        new SleepAction(.5),
                        releaseSpecimen(),
                        new SleepAction(.3),
                        new ParallelAction(ParkinDeck, //don't call stow; call wall position
                                new SequentialAction(
                                slidestozero(), rotatorarmstozero()))
                        //new SleepAction(5))
*/
        )   );
        drive.updatePoseEstimate();

        Geronimo.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Geronimo.autoTimeLeft);
        telemetry.update();

    }

    public Action grabSpecimenfromwall (){return new GrabSpecimenFromWall();}
    public class GrabSpecimenFromWall implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
               // control.limelightHasTarget();
                control.SpecimenPickupFromWallServoPosition();
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
                //control.SpecimenPickupFromWallServoPosition();
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
                control.RemoveFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidestozero (){return new SlidesToZero();}
    public class SlidesToZero implements Action {
        private boolean initialized = false;
        double timeout = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetSlideToPosition(0);
                initialized = true;
                timeout = control.opMode.getRuntime() + 0.5;
            }
            boolean returnValue = true;
            if (control.opMode.getRuntime()>=timeout || control.GetSlidesLimitSwitchPressed())
            {
                returnValue = false;
            }
            packet.put("lock purple pixel", 0);
            return returnValue ;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action rotatorarmstozero (){return new RotatorArmsToZero();}
    public class RotatorArmsToZero implements Action {
        private boolean initialized = false;
        double timeout = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetSlideRotatorArmToPosition(0);
                initialized = true;
                timeout = control.opMode.getRuntime() + 0.5;
            }
            boolean returnValue = true;
            if (control.opMode.getRuntime()>=timeout || control.GetSlideRotatorArmLimitSwitchPressed())
            {
                returnValue = false;
            }
            packet.put("lock purple pixel", 0);
            return returnValue ;

              // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action grabSpecimen (){return new GrabSpecimen();}
    public class GrabSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetClawPosition(Geronimo.CLAW_MAX_POS);
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action liftSpecimenoffWall (){return new LiftSpecimenOffWall();}
    public class LiftSpecimenOffWall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.RemoveFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action deliverSpecimenHigh (){return new DeliverSpecimenHigh();}
    public class DeliverSpecimenHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverHighChamberAlternate();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishdeliverSpecimenHigh (){return new FinishDeliverSpecimenHigh();}
    public class FinishDeliverSpecimenHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SpecimenDeliverHighChamberFinishingMove();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
}

