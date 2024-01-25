package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetPoseCmd extends Command {

    private final SwerveSys swerveSys;

    private final Pose2d pose;

    public SetPoseCmd(Pose2d pose, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.pose = pose;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveSys.setPose(pose);
        swerveSys.setHeading(pose.getRotation());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
