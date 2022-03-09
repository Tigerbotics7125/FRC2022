/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.robot.subsystems;

import static frc.robot.constants.LEDMatrixConstants.*;

import frc.tigerlib.ledmatrix.Matrix2D;

public class LEDMatrixSub {
    private Matrix2D m_matrix;

    public LEDMatrixSub() {
        try {
            m_matrix = new Matrix2D(kWidth, kHeight, kSerpentine, kPWMPort, kGifs);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
