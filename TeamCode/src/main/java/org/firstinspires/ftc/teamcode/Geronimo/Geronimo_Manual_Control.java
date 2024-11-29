package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Geronimo 1 Manual Control")
//@Disabled
public class Geronimo_Manual_Control extends LinearOpMode {
    Geronimo theRobot;
    boolean rotatorPowerApplied = false;

    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)

    public void runOpMode() {
        theRobot = new Geronimo(true, true, this);

        theRobot.Init(this.hardwareMap);
        theRobot.ShowTelemetry();
        telemetry.update();

        waitForStart();
        resetRuntime();
        theRobot.AutoStartPosition();

        while (opModeIsActive()) {
            theRobot.EndgameBuzzer();
            /* *************************************************
             *************************************************
             * Driver Controls (gamepad1)
             *************************************************
             *************************************************
             */
            // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
            theRobot.RunDriveControls();

            // Press the triangle button / "y" while facing directly away from the driver to set the IMU correctly for field-centric mode if off
            if (gamepad1.triangle && !gamepad1.options) {
                theRobot.imu.resetYaw();
            }
            // Press the triangle + options/start at same time to switch to field centric drive mode (default at startup)
            else if (gamepad1.triangle && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(true);
            }
            // Press the square + options/start at same time to switch to robot centric drive mode
            else if (gamepad1.square && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(false);
            }
            // gamepad_1.square --> stop and reset on the slides
            //else if (gamepad1.square) {
            //    theRobot.ResetSlidesPower();
            //}

            /* *************************************************
             *************************************************
             * Arm Controls (gamepad2)
             *
             *************************************************
             *************************************************
             */
            // SLIDE MOTOR CONTROL
            if ((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y <= -0.1)) {
                theRobot.SetSlidesPower(gamepad2.left_stick_y);
            } else {
                theRobot.SetSlidesPower(0.0);
            }

            // Slide Rotator Controls
            if (gamepad2.right_trigger > 0.25) {
                rotatorPowerApplied = true;
                theRobot.SetSlideRotatorToPowerMode(0.4);
            } else if (gamepad2.left_trigger > 0.25) {
                rotatorPowerApplied = true;
                theRobot.SetSlideRotatorToPowerMode(-0.4);
            }
            // if was using triggers and now let go, tell the arms to hold at this position
            else if (rotatorPowerApplied && !theRobot.GetRotatorArmRunningToPosition() && !theRobot.GetSlideRotatorArmLimitSwitchPressed()) {
                rotatorPowerApplied = false;
                theRobot.SetSlideRotatorArmToPosition(theRobot.GetRotatorLeftArmCurrentPosition());
            } else if (theRobot.GetRotatorArmRunningToPosition()) {
                if (theRobot.GetRotatorArmTargetPosition() == 0 && theRobot.GetSlideRotatorArmLimitSwitchPressed()) {
                    theRobot.ResetSlideRotatorArmToZero();

                } else {
                    int leftDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorLeftArmCurrentPosition());
                    int rightDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorRightArmCurrentPosition());
                    if ((leftDifference <= 2) && (rightDifference <= 2)) {
                        theRobot.SetSlideRotatorArmToHoldCurrentPosition();
                    }
                }
            }

            // Combo Moves
            if (gamepad2.cross && !gamepad2.options) {
                theRobot.IntakeFromFloor();
            } else if (gamepad2.share) {
                theRobot.AutoStartPosition();
            } else if (gamepad2.circle && !gamepad2.options) {
                theRobot.SpecimenDeliverLow();
            } else if (gamepad2.triangle && !gamepad2.options) {
                theRobot.SpecimenDeliverHigh();
            } else if (gamepad2.square && !gamepad2.options) {
                theRobot.SpecimenPickupFromWall();
            } else if (gamepad2.options && !gamepad2.circle && !gamepad2.cross && !gamepad2.triangle && !gamepad2.square) {
                theRobot.RemoveFromWall();
            } else if (gamepad2.dpad_up) {
                theRobot.BasketHigh();
            }
            // Claw Control
            else if (gamepad2.right_bumper) {
                theRobot.SetClawPosition(Geronimo.CLAW_MAX_POS);
            } else if (gamepad2.left_bumper) {
                theRobot.SetClawPosition(Geronimo.CLAW_MIN_POS);
            }
            // Green Box Control
            else if (gamepad2.dpad_left) {
                theRobot.SetIntakeBoxRotatorDecrementDown();
            } else if (gamepad2.dpad_right) {
                theRobot.SetIntakeBoxRotatorIncrementUp();
            }
            // Intake Stars Control -- Intake --> OFF --> Outtake --> OFF --> Intake
            else if (gamepad2.dpad_down) {
                theRobot.CycleIntakeStarMode();
            }
            // Small Arms (Hangers)
            else if (gamepad2.right_stick_x > 0.2) {
                theRobot.SetSmallArmHangerIncrementUp();
            } else if (gamepad2.right_stick_x < -0.2) {
                theRobot.SetSmallArmHangerDecrementDown();
            }

            theRobot.ShowTelemetry();
            telemetry.update();
        } // end while (opModeIsActive())

    }
}
