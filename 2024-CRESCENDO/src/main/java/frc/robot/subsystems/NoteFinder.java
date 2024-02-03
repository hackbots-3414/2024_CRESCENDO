// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gamepiece;
import frc.robot.Constants.NoteFinderConstants;

/**
 * Notefinder receives datagram packets for commands to use
 * [array of angles]|[array of confidences]|"quoted status message"
 * 512 bytes max payload
 * 1 decimal place
 * 
 * [-169.9, 169.9,0]|[100,99.1,70.0]|"This is a test.
 * `~!@#$%^&*()_+\{};:',.<>/?"
 * '|'[]' were deleted from dataset.
 */
public class NoteFinder extends SubsystemBase {
  private static final Logger LOG = LoggerFactory.getLogger(NoteFinder.class);
  private DatagramChannel noteChannel = null;
  private ByteBuffer byteReceiver = ByteBuffer.allocate(NoteFinderConstants.BUFFER_SIZE);
  private ArrayList<Gamepiece> gamepieces = new ArrayList<>();
  private StringBuffer status = new StringBuffer();
  private long lastUpdateTime = 0;
  StringBuilder stringBuilder = new StringBuilder(NoteFinderConstants.BUFFER_SIZE);

  /** Creates a new NoteFinder. */
  public NoteFinder() {
    // Trying to open a datagram Channel and set port
    try {
      noteChannel = DatagramChannel.open();
      noteChannel.configureBlocking(false);
      noteChannel.socket().bind(new InetSocketAddress(NoteFinderConstants.DATAGRAM_PORT));
      // Catching and Logging an error
    } catch (IOException ioe) {
      LOG.error("Failure to open note channel", ioe);
    }
  }

  // Synchronizing makes sure that Read and Write don't interfere with eachother
  public synchronized Gamepiece[] getGamepieces() {
    return gamepieces.toArray(new Gamepiece[gamepieces.size()]);
  }

  public String getStatus() {
    return status.toString();
  }

  public long getLastUpdateTime() {
    return lastUpdateTime;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // dataReceiver();
  }

  public void dataReceiver() {
    // Refreshes the Byte Reciever and assigns the byte to senderaddress
    try {
      byteReceiver.clear();
      SocketAddress senderAddress = noteChannel.receive(byteReceiver);
      if (senderAddress == null) {
        return;
      }
      // Tracing the recieve data and Sender address, and logging the error
      LOG.trace("receive data: {} Sender: {}", byteReceiver, senderAddress);
    } catch (Exception ioe) {
      LOG.error("Failure to receive data", ioe);
    }
    try {
      parseBuffer();
      LOG.trace("Updated Game Pieces: {}", gamepieces);
    } catch (Exception e) {
      LOG.error("Bad MESSAGE", e);
    }
  }

  private synchronized void parseBuffer() {
    // [Angle]|[Confidence]|"messsage"
    // Commas within Angle and Confidence indicate multiple gamepieces detected
    // [-169.9,169.9,0]|[100,99.1,70.0]|"This is a test."
    // Taking bytereceiver to beginning then clear gamepieces.
    byteReceiver.flip();
    gamepieces.clear();
    stringBuilder.setLength(0);
    char currentByte = 0;
    int commaCount = 0;
    // Copying message into a bytebuilder
    for (int i = 0; i < byteReceiver.limit(); i++) {
      currentByte = (char) byteReceiver.get();
      if (currentByte == ',') {
        commaCount++;
      }
      stringBuilder.append(currentByte);
    }
    // Dividing CommaCount by 2 to get the number of gamepieces
    commaCount = commaCount / 2;
    String firstString = null;
    String secondString = null;
    String thirdString = null;
    // Assigning variable to define the start and end of the message parts
    firstString = stringBuilder.substring(1, stringBuilder.indexOf("]"));
    secondString = stringBuilder.substring(stringBuilder.indexOf("|") + 2, stringBuilder.lastIndexOf("]"));
    thirdString = stringBuilder.substring(stringBuilder.indexOf("\"") + 1, stringBuilder.lastIndexOf("\""));
    // Logging the data in the right order
    LOG.trace("First String: {} Second String: {} Third String: {}", firstString, secondString, thirdString);
    status.setLength(0);
    status.append(thirdString);
    if (firstString.length() == 0) {
      lastUpdateTime = System.currentTimeMillis();
      return;
    }
    double[] angleArray = parseDoubleArray(firstString, commaCount);
    double[] confidenceArray = parseDoubleArray(secondString, commaCount);
    Gamepiece gamepiece = null;
    for (int i = 0; i < angleArray.length; i++) {
      gamepiece = new Gamepiece();
      gamepiece.setAngle(angleArray[i]);
      gamepiece.setConfidence(confidenceArray[i]);
      gamepieces.add(gamepiece);
    }
    lastUpdateTime = System.currentTimeMillis();
  }

  private double[] parseDoubleArray(String stringIn, int commaCount) {
    double[] returnArray = new double[commaCount + 1];
    int startVar = 0;
    int endVar = 0;
    for (int i = 0; i < returnArray.length; i++) {
      endVar = stringIn.indexOf(",", startVar);
      if (endVar < 0) {
        endVar = stringIn.length();
      }
      returnArray[i] = Double.valueOf(stringIn.substring(startVar, endVar));
      startVar = endVar + 1;
    }
    return returnArray;
  }
}