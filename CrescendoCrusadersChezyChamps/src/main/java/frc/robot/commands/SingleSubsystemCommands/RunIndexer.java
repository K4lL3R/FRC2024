package frc.robot.commands.SingleSubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends Command {
    private Indexer s_Indexer;
    private double speed;

    public RunIndexer(Indexer indexer, double speed) {
        s_Indexer = indexer;
        this.speed = speed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        s_Indexer.setIndexerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
