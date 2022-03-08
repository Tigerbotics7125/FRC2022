/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib.ledmatrix;

import java.awt.image.BufferedImage;

/** from here https://stackoverflow.com/a/18425922/16294208 */
public class ImageFrame {
    private final int delay;
    private final BufferedImage image;
    private final String disposal;

    public ImageFrame(BufferedImage image, int delay, String disposal) {
        this.image = image;
        this.delay = delay;
        this.disposal = disposal;
    }

    public BufferedImage getImage() {
        return image;
    }

    public int getDelay() {
        return delay;
    }

    public String getDisposal() {
        return disposal;
    }
}
