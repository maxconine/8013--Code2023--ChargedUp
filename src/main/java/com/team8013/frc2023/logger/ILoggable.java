package com.team8013.frc2023.logger;
import java.util.ArrayList;

/*  
    See:
         LogStorage for functionality
         TestLoggable for implementation
*/
 
 public interface ILoggable {
     ArrayList<ArrayList<Number>> getItems();
     ArrayList<String> getItemNames();
 }
