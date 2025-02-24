package FtcRobotController.MeepMeepTesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingFr {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10.6, 63.2, 0))
                ///When Actions are implemented, this auto should be able to do a 50 point auto on blue
                .setTangent(0)
                ///Score preloaded speci on high chamber
                .strafeToSplineHeading(new Vector2d(-7.5, 32), Math.toRadians(90))
                //.Action.SlideSpecScore
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(-7.5, 36))


                ///Sample Sweep into obv zone
                .strafeTo(new Vector2d(-7.5, 42))
                .strafeToLinearHeading(new Vector2d(-34, 25), Math.toRadians(180))
                .strafeTo(new Vector2d(-36, 25)) //Add VelConstraints so piece isn't rammed into
                //.Action.Sweep
                .waitSeconds(1)
                .strafeTo(new Vector2d(-43, 25))
                .waitSeconds(1)
                //.Action.Sweep
                .strafeTo(new Vector2d(-54, 25))
                .waitSeconds(1)
                //.Action.Sweep
                .waitSeconds(1)
                ///Obv Run
                .strafeToLinearHeading(new Vector2d(-45, 50), Math.toRadians(270))
                .strafeTo(new Vector2d(-45, 60))

                ///HighChamber Run
                .strafeToSplineHeading(new Vector2d(-6.5, 32), Math.toRadians(90))
                //.Action.SlideSpecScore
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(-6.5, 36))
                ///Obv Run
                .strafeToLinearHeading(new Vector2d(-45, 50), Math.toRadians(270))
                .strafeTo(new Vector2d(-45, 60))

                ///HighChamber Run
                .strafeToSplineHeading(new Vector2d(-5.5, 32), Math.toRadians(90))
                //.Action.SlideSpecScore
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(-5.5, 36))
                ///Obv Run
                .strafeToLinearHeading(new Vector2d(-45, 50), Math.toRadians(270))
                //.Action.SlideSpecGrab
                .strafeTo(new Vector2d(-45, 60))


                ///HighChamber Run
                .strafeToSplineHeading(new Vector2d(-4.5, 32), Math.toRadians(90))
                //.Action.SlideSpecScore
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(-4.5, 36))

                ///Obv Run
                .strafeToLinearHeading(new Vector2d(-45, 50), Math.toRadians(270))
                //.Action.SlideSpecGrab
                .strafeTo(new Vector2d(-45, 60))

                ///HighChamber Run
                .strafeToSplineHeading(new Vector2d(-3.5, 32), Math.toRadians(90))
                //.Action.SlideSpecScore
                .waitSeconds(0.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}