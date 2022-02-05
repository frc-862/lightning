// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.common.command.core;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Deletes RoboRio log files
 */
public class DeleteLogCommand extends CommandBase {

  private int filesToKeep;

  private boolean isDone;

  // Default log directory in Roborio
  File logFolder = new File("/home/lvuser/log"); 
  File logFiles[] = logFolder.listFiles();
  
  /**
   * Commmand that deletes a certain number of log files, starting from the oldest
   * @param filesToKeep Number of files to keep in the RoboRio
   */
  public DeleteLogCommand(int filesToKeep) {
    this.filesToKeep = filesToKeep;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("logFileCount", getLogAmount());
    Arrays.sort(logFiles);

    if (logFiles.length > filesToKeep) {
      // Calculating number of files to delete
      int overcount = logFiles.length - filesToKeep;
      
      for (int i = 0; i < overcount; i++) {
        logFiles[i].delete();
      }

      logFiles = logFolder.listFiles();

      // Renaming log files to "DeleteLogCommands-<i + 1>05-dl.log"
      for (int i = 0; i < filesToKeep; i++) {
        logFiles[i].renameTo(new File(String.format("/home/lvuser/log/%s-%05d-dl.log", getClass().getSimpleName(), i + 1)));
      }
    }
    isDone = true;
  }

  public int getLogAmount() {
    return logFiles.length;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
