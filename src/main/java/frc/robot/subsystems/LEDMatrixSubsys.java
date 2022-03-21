/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.Constants.LEDMatrix.kGifs;
import static frc.robot.Constants.LEDMatrix.kHeight;
import static frc.robot.Constants.LEDMatrix.kPWMPort;
import static frc.robot.Constants.LEDMatrix.kSerpentine;
import static frc.robot.Constants.LEDMatrix.kWidth;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.tigerlib.ledmatrix.Matrix2D;

public class LEDMatrixSubsys extends SubsystemBase {
    private Matrix2D m_matrix;

    public LEDMatrixSubsys() {
        try {
            m_matrix = new Matrix2D(kWidth, kHeight, kSerpentine, kPWMPort, kGifs);
            m_matrix.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
