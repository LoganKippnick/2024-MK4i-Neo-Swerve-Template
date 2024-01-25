package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetHeadingCmd extends Command {

    private final SwerveSys swerveSys;

    private final Rotation2d heading;

    public SetHeadingCmd(Rotation2d heading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.heading = heading;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveSys.setHeading(heading);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
