package frc.robot.autos;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve, String pathName){
        try {
            String relativePath = "pathplanner/generatedJSON/" + pathName + ".wpilib.json";
            Path path = Filesystem.getDeployDirectory().toPath().resolve(relativePath);

            Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(path);

            var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    exampleTrajectory,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);


            addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())), swerveControllerCommand);
        } catch (Exception e) {
            System.out.println("Path cannot be loaded");   
        }
    }
}