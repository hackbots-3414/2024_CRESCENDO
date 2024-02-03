// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.IOException;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import frc.robot.subsystems.NoteFinder;
import frc.robot.Constants.NoteFinderConstants;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.api.BeforeAll;

/** Add your docs here. */
@TestInstance(Lifecycle.PER_CLASS)
public class NoteFinderTest {
    private NoteFinder noteFinder = new NoteFinder();
    private DatagramChannel noteChannel = null;
    private ByteBuffer byteSender = null;

    public NoteFinderTest() throws IOException {
    }

    @BeforeAll 
    public void setup() throws IOException {
        noteChannel = DatagramChannel.open();
        byteSender = ByteBuffer.allocate(NoteFinderConstants.BUFFER_SIZE);
    }

    private int sendMessage(String message) throws IOException {
        byteSender.clear();
        byteSender.put(message.getBytes());
        byteSender.flip();
        int bytesSent = noteChannel.send(byteSender,
                new InetSocketAddress("127.0.0.1", NoteFinderConstants.DATAGRAM_PORT));
        noteFinder.dataReceiver();
        return bytesSent;

    }

    private int sendMessageNoData(String message) throws IOException {
        byteSender.clear();
        byteSender.put(message.getBytes());
        byteSender.flip();
        int bytesSent = noteChannel.send(byteSender,
                new InetSocketAddress("127.0.0.1", NoteFinderConstants.DATAGRAM_PORT));
        return bytesSent;

    }

    private void testLastUpdate() {
        assertTrue(noteFinder.getLastUpdateTime() > System.currentTimeMillis() - 25, "Testing LastUpdateTime");
    }

    @Test
    public void testZeroNotes() throws IOException {
        sendMessage("[]|[]|\"No Game Piece\"");
        assertEquals("No Game Piece", noteFinder.getStatus(), "Status message finder");
        assertEquals(0, noteFinder.getGamepieces().length, " Number of Gamepieces");
        System.out.println("Now - LastUpdateTime: " + (System.currentTimeMillis() - noteFinder.getLastUpdateTime()));
        testLastUpdate();
    }

    @Test
    public void testOneNote() throws IOException {
        sendMessage("[162]|[62]|\"One Game Piece\"");
        assertEquals("One Game Piece", noteFinder.getStatus(), "Status message finder");
        assertEquals(1, noteFinder.getGamepieces().length, " Number of Gamepieces");
        System.out.println("Now - LastUpdateTime: " + (System.currentTimeMillis() - noteFinder.getLastUpdateTime()));
        testLastUpdate();
        assertEquals(162.0, noteFinder.getGamepieces()[0].getAngle(), "Angle Tester");
        assertEquals(62.0, noteFinder.getGamepieces()[0].getConfidence(), "Confidence Tester");
    }

    @Test
    public void testSecondNote() throws IOException {
        sendMessage("[149,13]|[12,81]|\"Two Game Pieces   \"");
        assertEquals("Two Game Pieces   ", noteFinder.getStatus(), "Status message finder");
        assertEquals(2, noteFinder.getGamepieces().length, " Number of Gamepieces");
        System.out.println("Now - LastUpdateTime: " + (System.currentTimeMillis() - noteFinder.getLastUpdateTime()));
        testLastUpdate();
        assertEquals(149, noteFinder.getGamepieces()[0].getAngle(), "Angle Tester");
        assertEquals(13, noteFinder.getGamepieces()[1].getAngle(), "Angle Tester");
        assertEquals(12, noteFinder.getGamepieces()[0].getConfidence(), "Confidence Tester");
        assertEquals(81, noteFinder.getGamepieces()[1].getConfidence(), "Confidence Tester");
    }

     @Test
    public void testThreeNote() throws IOException {
        sendMessage("[41,-114,93]|[31,71,-94]|\"Three Game Pieces   \"");
        assertEquals("Three Game Pieces   ", noteFinder.getStatus(), "Status message finder");
        assertEquals(3, noteFinder.getGamepieces().length, " Number of Gamepieces");
        System.out.println("Now - LastUpdateTime: " + (System.currentTimeMillis() - noteFinder.getLastUpdateTime()));
        testLastUpdate();
        assertEquals(41, noteFinder.getGamepieces()[0].getAngle(), "Angle Tester");
        assertEquals(-114, noteFinder.getGamepieces()[1].getAngle(), "Angle Tester");
        assertEquals(93, noteFinder.getGamepieces()[2].getAngle(), "Angle Tester");
        assertEquals(31, noteFinder.getGamepieces()[0].getConfidence(), "Confidence Tester");
        assertEquals(71, noteFinder.getGamepieces()[1].getConfidence(), "Confidence Tester");
        assertEquals(-94, noteFinder.getGamepieces()[2].getConfidence(), "Confidence Tester");
    }

    @Test
    public void pastTest() throws IOException { 
        sendMessageNoData("[149,13]|[12,81]|\"Two Game Pieces   \"");
        sendMessageNoData("[41,-114,93]|[31,71,-94]|\"Three Game Pieces   \"");
        // This is to make sure that we get the latest value
        noteFinder.dataReceiver();
        noteFinder.dataReceiver();
        assertEquals(3, noteFinder.getGamepieces().length, " Number of Gamepieces");
    }
}