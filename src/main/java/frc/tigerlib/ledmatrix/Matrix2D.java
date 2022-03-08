/**
 * Copyright (C) 2022, Tigerbotics' team members and all other contributors.
 * Open source software; you can modify and/or share this software.
 */
package frc.tigerlib.ledmatrix;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.HashSet;

public class Matrix2D extends Thread {
    private Thread m_thread;
    private AddressableLED m_leds;
    private int m_width;
    private int m_height;
    private boolean m_isSerpentine;
    private HashSet<Gif> m_gifs;

    /**
     * @param width The width (X axis) in terms of individual leds.
     * @param height The height (Y axis) in terms of individual leds.
     * @param isSerpentine Is the layout serpentine?
     *     <pre><code>
     * Serpentine False example:
     *  Each line begins and ends in the same direction.
     * //     0 >  1 >  2 >  3 >  4
     * //                        |
     * //    .----<----<----<----'
     * //    |
     * //    5 >  6 >  7 >  8 >  9
     * //                        |
     * //    .----<----<----<----'
     * //    |
     * //   10 > 11 > 12 > 13 > 14
     * //                        |
     * //    .----<----<----<----'
     * //    |
     * //   15 > 16 > 17 > 18 > 19
     *
     * Serpentine True example:
     *  Each line alternates which direction where it begins and ends.
     * //    0 >  1 >  2 >  3 >  4
     * //                        |
     * //                        |
     * //    9 <  8 <  7 <  6 <  5
     * //    |
     * //    |
     * //   10 > 11 > 12 > 13 > 14
     * //                       |
     * //                       |
     * //   19 < 18 < 17 < 16 < 15
     * </code></pre>
     *
     * @param brightness The brightness to set the leds at (0 - 255)
     * @param pwmPort the PWM port the LEDs are plugged into.
     * @throws Exception
     */
    public Matrix2D(int width, int height, boolean isSerpentine, int pwmPort, Gif... gifs)
            throws Exception {
        if ((width * height > 5460) || (width * height <= 0))
            throw new Exception(
                    "Incompatible size, width and/or height incompatible; maximum 5460 LEDs.");
        if (pwmPort < 0 || pwmPort > 9)
            throw new Exception("Incompatible PWM port, must be [0..9].");

        m_thread = this;

        this.m_width = width;
        this.m_height = height;
        this.m_isSerpentine = isSerpentine;

        m_leds = new AddressableLED(pwmPort);
        m_leds.setLength(m_width * m_height);

        for (Gif gif : gifs) {
            m_gifs.add(gif);
        }

        m_thread.start();
    }

    @Override
    public void run() {
        for (Gif gif : m_gifs) {
            try {
                displayGif(gif);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Displays the given gif.
     *
     * @param gif
     * @throws Exception
     */
    public void displayGif(Gif gif) throws Exception {
        checkCompatibility(gif);

        // turn off leds so that the entire matrix is clean.
        m_leds.stop();

        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_width * m_height);

        // step through frames
        for (Integer[][][] frame : gif.frames) {
            for (int column = 0; column < m_height; column++) {
                for (int row = 0; row < m_width; row++) {
                    Color8Bit c =
                            new Color8Bit(
                                    frame[row][column][0],
                                    frame[row][column][1],
                                    frame[row][column][2]);
                    buffer.setLED(getLedPos(row, column, m_isSerpentine), c);
                }
            }

            m_leds.setData(buffer);
            Thread.sleep(gif.frameDelay * 10); // idk why its 10 *shrug*.
        }
    }

    /*
     * @returns an array index for the led single array from the matrix double array
     */
    private int getLedPos(int x, int y, boolean isSerpentine) {
        // https://forum.arduino.cc/t/ws2811-10-x-10-led-matrix-serpentine-wiring/606106/2
        if (isSerpentine && y % 2 != 0) { // if odd row
            return (y * this.m_width) + (this.m_width - 1 - x);
        } else { // even row or no serpentine
            return x + (y * m_width);
        }
    }

    /**
     * Checks if the givin gif is of the same width and height of the matrix.
     *
     * @param gif the gif to check.
     * @throws Exception
     */
    private void checkCompatibility(Gif gif) throws Exception {
        if (gif.height != this.m_height || gif.width != this.m_width)
            throw new Exception("Incompatible gif; size of gif and matrix must match.");
    }

    /**
     * Add the given gifs if not already in the HashSet.
     *
     * @param gifs The gifs to add.
     * @returns If the set was changed or not.
     */
    public boolean addGifs(Gif... gifs) {
        boolean change = false;
        for (Gif gif : gifs) {
            if (m_gifs.add(gif)) change = true;
        }
        return change;
    }

    /**
     * Remove the givin gifs if contained in the HashSet.
     *
     * @param gifs The gifs to remove.
     * @returns If the set was changed or not.
     */
    public boolean removeGifs(Gif... gifs) {
        boolean change = false;
        for (Gif gif : gifs) {
            if (m_gifs.remove(gif)) change = true;
        }
        return change;
    }
}
