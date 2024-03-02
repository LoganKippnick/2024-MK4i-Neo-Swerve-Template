// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetInitialPoseCmd extends Command {

private final SwerveSys swerveSys;

private PathPlannerPath firstPath;

/**
 * Creates a new SetInitialPoseCmd.
 * <p>
 * SetInitialPoseCmd sets the pose of the robot to the starting pose of a specified trajectory.
 * This should only be used to set position at the start of an autonomous program.
 * 
 * @param firstPathName The name of the trajectory given in PathPlanner to take the initial pose of.
 * @param swerveSys The SwerveSys to set the initial pose.
 */
public SetInitialPoseCmd(String firstPathName, SwerveSys swerveSys) {
		firstPath = PathPlannerPath.fromPathFile(firstPathName);

		this.swerveSys = swerveSys;

		addRequirements(swerveSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if(DriverStation.getAlliance().get() == Alliance.Red) {
			firstPath = firstPath.flipPath();
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// swerveSys.setTranslation(firstPath.getPreviewStartingHolonomicPose().getTranslation());
		
		// swerveSys.setTranslation(new Translation2d(1.3, 5.55));

		// swerveSys.setHeading(firstPath.getPreviewStartingHolonomicPose().getRotation());

		swerveSys.setPose(firstPath.getPreviewStartingHolonomicPose());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
