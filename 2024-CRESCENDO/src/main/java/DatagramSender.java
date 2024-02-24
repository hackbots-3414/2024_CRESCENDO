// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import frc.robot.Constants.NoteFinderConstants;

/** Add your docs here. */
public class DatagramSender {

    public static void main(String[] args) throws Exception {
        System.out.println(args.length);
        DatagramChannel noteChannel = DatagramChannel.open();
        ByteBuffer byteSender = ByteBuffer.allocate(NoteFinderConstants.BUFFER_SIZE);
        String message = "[-169.9, 169.9,0]|[100,99.1,70.0]|\"This is a test.\"";
        if(args.length > 0) {
            message = args[0];
        }
        byteSender.clear();
        System.out.println(message);
        byteSender.put(message.getBytes());
        byteSender.flip();

        int bytesSent = noteChannel.send(byteSender, new InetSocketAddress("127.0.0.1", NoteFinderConstants.DATAGRAM_PORT));
        System.out.println("bytesSent: "+bytesSent);
    }

}