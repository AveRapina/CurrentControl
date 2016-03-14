/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import gnu.io.UnsupportedCommOperationException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.TooManyListenersException;

//package vi;




/**
 *
 * @author Nelson
 */
public class Comunications {
    
    // private vars
    private SerialPort serialPort;
    private SerialPortEventListener listener;
    private OutputStream outputStream;
    private InputStream inputStream;
    private boolean connected;
    
    
    
    // methods
    
    public boolean connect(final String name){
        if(connected)
            return true;
        final Enumeration<CommPortIdentifier> portList=CommPortIdentifier.getPortIdentifiers();
        CommPortIdentifier portId =null;
        
        while (portList.hasMoreElements()){
            portId = portList.nextElement();
            if(portId.getPortType() == CommPortIdentifier.PORT_SERIAL){
                System.out.println("serial " + portId.getName());
                if(portId.getName().equals(name)){
                    break;
                }
                
            }
        }
        if(portId == null)
            return false;
        
        try{
            serialPort =(SerialPort) portId.open("SimpleReadApp", 2000);
            inputStream =serialPort.getInputStream();
            outputStream =serialPort.getOutputStream();
            // neww listener
            listener =new SerialPortEventListener() {

                @Override
                public void serialEvent(SerialPortEvent spe) {
                    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
                }
            };
            serialPort.notifyOnDataAvailable(true);
            serialPort.setSerialPortParams(9600,SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
        }catch (final IOException e){
            e.printStackTrace();
        }
        
        new Thread((Runnable) listener).start();
        connected =true;
        return true;
            
        
        
    }
    
    
    public void disconnect(){
        try{
            listener.stop =true;
            connected=false;
            outputStream.close();
            inputStream.close();
            
            
        }catch(){
            
        }
    }
    
    
        
    
}
