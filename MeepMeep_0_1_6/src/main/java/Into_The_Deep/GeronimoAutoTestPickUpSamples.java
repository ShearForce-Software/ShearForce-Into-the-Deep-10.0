package Into_The_Deep;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class GeronimoAutoTestPickUpSamples {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

       // static Pose2d startPose;
    // Testing from home


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d StartPose = new Pose2d(-12, -63, Math.toRadians(90));
        Vector2d VectorTwo = new Vector2d(-12, -36);
        myBot.runAction(myBot.getDrive().actionBuilder(StartPose)
                           //  .splineToLinearHeading(new Pose2d(36,48,36.6), Math.toRadians(270))
                                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                                 .strafeTo(new Vector2d(-12, -40))
                                .splineToLinearHeading(new Pose2d(-48,-29, Math.toRadians(180)), Math.toRadians(180))
                       //         .strafeToLinearHeading(new Vector2d(-48, -29), Math.toRadians(180))
                             //    .strafeToLinearHeading(new Vector2d(-12, -36), Math.toRadians(225))
                                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                     //   .splineToLinearHeading(new Pose2d(-53, -29, Math.toRadians(90)), Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(-53, -29), Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                                .strafeToLinearHeading(new Vector2d(-60,-48), Math.toRadians(90))
                               .strafeToLinearHeading(new Vector2d(-62, -48), Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(-62, -29), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-60, -29), Math.toRadians(90))
                              .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                              //  .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                /*
                                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                                .strafeToLinearHeading(VectorTwo, Math.toRadians(90))

                                */

                // assume using some sensor to sense basket and deliver

                 /* .turn(Math.toRadians(270))
                  .lineToX(0)
                  .turn(Math.toRadians(90))
                  .turn(Math.toRadians(90))
                  .lineToY(-24)

                  */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
