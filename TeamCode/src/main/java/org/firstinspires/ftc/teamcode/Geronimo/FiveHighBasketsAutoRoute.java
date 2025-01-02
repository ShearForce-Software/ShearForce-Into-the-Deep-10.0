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


@Autonomous(name="Five High Baskets Auto Route", preselectTeleOp =
        "Geronimo 1 Manual Control")
// @Disabled
public class FiveHighBasketsAutoRoute extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;

    // Trajectories
    Action DeliverStartingSample;
    Action DriveToSample1;

    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint normalVelocityConstraint;
    AccelConstraint normalAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    VelConstraint intakeVelocityConstraint;
    VelConstraint humanPlayerVelocityConstraint;

    public void runOpMode(){
        startPose = new Pose2d(- 12,-64, Math.toRadians(90));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);


        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        //  -- need to adjust this starting position to keep the specimen out of the wall Check
        control.Stow();
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

        DeliverStartingSample = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, -36),Math.toRadians(90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-57,-57), Math.toRadians(45))
                .build();
        DriveToSample1 = drive.actionBuilder(new Pose2d(-57,-57,Math.toRadians(45)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-36,-36), Math.toRadians(90))
                .build();



        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        //Drive to submersible and pick up sample
                        new ParallelAction(closeUrchin(),DeliverStartingSample, basketHigh()),
                        // Raise slides to high basket height
                        finishBasketHigh_SlidesPosition(),
                        new SleepAction(4.0),
                        // Rotate urchin to align above basket
                        finishBasketHigh_UrchinDeliverPosition(),
                        new SleepAction(0.4),
                        // Release the sample from the urchin
                        openUrchin(),
                        new SleepAction(0.4),
                        // Rotate urchin back away from the basket
                        finishBasketHigh_UrchinSafeToLowerPosition(),
                        new SleepAction(0.4),
                        // Rotate arms a little away from basket and lower slides to zero
                        finishBasketHigh_ArmSafeToLowerPosition(),
                        slidesToZero(),
                        rotatorarmstozero(),
                        DriveToSample1
                ));


        drive.updatePoseEstimate();

        Geronimo.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Geronimo.autoTimeLeft);
        telemetry.update();

    }

    public Action basketHigh (){return new BasketHigh();}
    public class BasketHigh implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
               // control.limelightHasTarget();
                control.BasketHigh();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

    public Action finishBasketHigh_SlidesPosition (){return new FinishBasketHigh_SlidesPosition();}
    public class FinishBasketHigh_SlidesPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_SlidesPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_UrchinDeliverPosition (){return new FinishBasketHigh_UrchinDeliverPosition();}
    public class FinishBasketHigh_UrchinDeliverPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_UrchinDeliverPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_UrchinSafeToLowerPosition (){return new FinishBasketHigh_UrchinSafeToLowerPosition();}
    public class FinishBasketHigh_UrchinSafeToLowerPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_UrchinSafeToLowerPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action finishBasketHigh_ArmSafeToLowerPosition (){return new FinishBasketHigh_ArmSafeToLowerPosition();}
    public class FinishBasketHigh_ArmSafeToLowerPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHighFinishingMove_ArmSafeToLowerPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action openUrchin (){return new OpenUrchin();}
    public class OpenUrchin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetUrchinServoPosition(0);
                //control.SpecimenPickupFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action closeUrchin (){return new CloseUrchin();}
    public class CloseUrchin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SetUrchinServoPosition(1);
                //control.SpecimenPickupFromWallServoPosition();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesToZero(){return new SlidesToZero();}
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
}

