package server;

import java.io.*;
import java.net.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.nio.file.FileSystems;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.json.*;
import java.util.List;
import org.springframework.stereotype.Service;

@Service
public class Server implements Runnable {
  private static final int TELEMETRY_PORT = 9090;
  private static final int DEBUG_PORT = 7070;

  private Socket telemetryClient; // TCP socket to pod
  private Process debugProcess;
  private Process compileProcess;

  // Telemetry
  private JSONObject telemetryData;
  private boolean telemetryConnected = false;

  // Debug
  private String searchPhrase;
  private String logTypeFilter;
  private String submoduleFilter;
  private int curStart;
  private int curEnd;
  private boolean isLive = true;
  private boolean moreLines;
  private List<String> logTypes = new ArrayList<>(List.of(""));
  private List<String> submoduleTypes = new ArrayList<>(List.of(""));
  private JSONArray terminalOutput = new JSONArray();

  // Compile
  private static final String COMPILE = "COMPILE";
  private static final String COMPILING = "COMPILING";
  private static final String COMPILED = "RECOMPILE";
  private static final String RETRY = "RETRY";
  
  private static final String IS_COMPILED = "isCompiled";
  private static final String IS_SUCCESS = "isSuccess";
  private static final String ERROR_MESSAGE = "errorMessage";
 
  private boolean isCompiling = false;  // A boolean indicating if the pod code is still compiling, mainly for the UI
  private String debugStatus = COMPILE;

  private JSONArray debugOutput = new JSONArray();
  private JSONObject debugData;

  @Override
  public void run() {
    // Initialize the compiling state
    initDebugData();

    ServerSocket telemetryServer = getServerSocket(TELEMETRY_PORT);
    System.out.println("Server now listening on port " + TELEMETRY_PORT);

    while (true) {
      System.out.println("Waiting to connect to client...");
      telemetryClient = getTelemetryClient(telemetryServer);
      telemetryConnected = true;
      System.out.println("Connected to telemetry client");

      Thread telemetryMessageReaderWorker = new Thread(new TelemetryMessageReader());

      telemetryMessageReaderWorker.start();
      try {
        telemetryMessageReaderWorker.join();
      } catch (InterruptedException e) {
        System.out.println("Problem joining telemetryMessageReaderWorker threads");
      }
      closeClient(telemetryClient);
    }
    // closeServer(telemetryServer);
  }

  private class StreamGobblerRun extends Thread {
    InputStream is;

    private StreamGobblerRun(InputStream is) {
      this.is = is;
    }

    @Override
    public void run() {
      try (Scanner scan = new Scanner(is)) {
        String line = null;
        while (scan.hasNextLine()) {
          JSONObject output = parseDebugOutput(scan.nextLine());
          terminalOutput.put(output);
        }
      }
      catch (Exception e) {
        System.out.println(e);
      }
    }
  }

  private class StreamGobblerCompile extends Thread {
    InputStream is;

    private StreamGobblerCompile(InputStream is) {
      this.is = is;
    }

    @Override
    public void run() {
      try (Scanner scan = new Scanner(is)) {
        String line = null;
        while (scan.hasNextLine()) {
          debugOutput.put(scan.nextLine());
        }
      }
      catch (Exception e) {
        System.out.println(e);
      }
    }
  }

  private JSONObject parseDebugOutput(String line) {
    JSONObject obj = new JSONObject();
    //                                     h       m     s        ms     dbg    [submodule]   log
    Pattern pattern = Pattern.compile("(\\d{2}:\\d{2}:\\d{2}\\.\\d{3}) (\\w*)\\[([\\w-]*)\\]: (.*)");
    Matcher matcher = pattern.matcher(line);

    if (matcher.find()) {
      obj.put("time", matcher.group(1));
      obj.put("log_type", matcher.group(2));
      obj.put("submodule", matcher.group(3));
      obj.put("log", matcher.group(4));
    }
    
    obj.put("line", line);
    return obj;
  }

  //Compiles the pod code
  public void debugCompile() {
    initDebugData();
    debugStatus = COMPILING;
    isCompiling = true; // The compiling process start

    String DIR_PATH = FileSystems.getDefault().getPath("./").toAbsolutePath().toString();
    String HYPED_PATH = DIR_PATH.substring(0, DIR_PATH.length() - 1) + "hyped-pod_code";

    ArrayList<String> command = new ArrayList<String>();
    // Suppose to be make -j, but it's so buggy so leave it as make for now
    command.add("make");

    try {
      System.out.println("Compiling from: " + HYPED_PATH);
      debugOutput = new JSONArray();
      compileProcess = new ProcessBuilder(command).directory(new File(HYPED_PATH)).start();

      StreamGobblerCompile errorGobbler = new StreamGobblerCompile(compileProcess.getErrorStream());
      StreamGobblerCompile outputGobbler = new StreamGobblerCompile(compileProcess.getInputStream());
          
      // start gobblers
      outputGobbler.start();
      errorGobbler.start();

      compileProcess.waitFor();

      boolean isSuccess = compileProcess.exitValue() == 0;

      //Procedure to check wheter hyped code is successfully compiled
      boolean isFileExisted = isHypedExist(); // For first time compilation

      debugData.put(IS_COMPILED, true);

      //Change the debugStatus
      if (isFileExisted) {
        debugStatus = COMPILED;
        //Ends the compiling process
      } else if (isCompiling){
        debugStatus = COMPILING;
      } else {
        debugStatus = RETRY;
      }

      // Update the debugData
      if (!isFileExisted) {
        // Fail when compile for the first time
        debugStatus = RETRY;
        debugData.put(IS_SUCCESS, false);
        convertDebugOutput();
      } else if (!isSuccess){
        // Normal fail
        debugStatus = RETRY;
        debugData.put(IS_SUCCESS, false);
        convertDebugOutput();
      } else {
        debugData.put(IS_SUCCESS, true);
      }
      isCompiling = false;
      System.out.println("Finish compiling");
    } catch (Throwable t) {
      t.printStackTrace();
    }

  }

  public void convertDebugOutput() {
    StringBuffer errorMessage = new StringBuffer("");

    for(int i=0; i < debugOutput.length(); i++) {
      errorMessage.append(debugOutput.getString(i));
      errorMessage.append("\n");
    }

    debugData.put(ERROR_MESSAGE, errorMessage.toString());
  }

  public String getDebugData() {
    if (debugData == null) {
      return null;
    }

    if (isHypedExist()) {
      debugData.put(IS_COMPILED, true);
    } else {
      debugData.put(IS_COMPILED, false);
    }

    //System.out.println(debugData.toString());
    return debugData.toString();
  }

  public String getDebugStatus() {
    if (isCompiling) {
      debugStatus = COMPILING;
    } else if (!isHypedExist() && Boolean.TRUE.equals(debugData.get(IS_SUCCESS))){
      debugStatus = COMPILE;
      isCompiling = false;
    }

    return debugStatus;
  }
  
  public boolean isHypedExist() {
    String DIR_PATH = FileSystems.getDefault().getPath("./").toAbsolutePath().toString();
    String HYPED_PATH = DIR_PATH.substring(0, DIR_PATH.length() - 1) + "hyped-pod_code/hyped"; // change this part for RELEASE

    File HYPED_CODE = new File(HYPED_PATH);
    return HYPED_CODE.exists();
  }

  public void initDebugData() {
    debugData = new JSONObject();
    debugData.put(IS_SUCCESS, true);
    debugData.remove(ERROR_MESSAGE);

    if (isHypedExist()) {
      debugStatus = COMPILED;
      debugData.put(IS_COMPILED, true);

    } else {
      debugStatus = COMPILE;

      debugData.put(IS_COMPILED, false);
    }
  }

  public void debugRun(JSONArray flags) {
    String DIR_PATH = FileSystems.getDefault().getPath("./").toAbsolutePath().toString();
    String HYPED_PATH = DIR_PATH.substring(0, DIR_PATH.length() - 1) + "hyped-pod_code"; // change this part for RELEASE
    String os = System.getProperty("os.name").substring(0,3);

    ArrayList<String> command = new ArrayList<String>();

    if(!os.equals("Mac")){
      command.add("stdbuf");
      command.add("-oL");
    }

    command.add("./hyped");
    for (int i = 0, size = flags.length(); i < size; i++) {
      String flag = flags.getString(i);
      command.add(flag);
    }
    command.add("-v");
    command.add("--debug=3");

    try {
      System.out.println("Running from: " + HYPED_PATH);
      terminalOutput = new JSONArray();
      debugProcess = new ProcessBuilder(command).directory(new File(HYPED_PATH)).start();

      StreamGobblerRun errorGobbler = new StreamGobblerRun(debugProcess.getErrorStream());
      StreamGobblerRun outputGobbler = new StreamGobblerRun(debugProcess.getInputStream());
          
      // start gobblers
      outputGobbler.start();
      errorGobbler.start();

    } catch (Throwable t) {
      t.printStackTrace();
    }
  }

  public void debugKill() {
    if (debugProcess == null) {
      return;
    }
    debugProcess.destroy();
  }

  public void debugUpdateSearchPhrase(String searchPhrase) {
    this.searchPhrase = searchPhrase.toLowerCase();
  }

  public void debugUpdateLogTypeFilter(String logTypeFilter) {
    this.logTypeFilter = logTypeFilter;
  }

  public void debugUpdateSubmoduleFilter(String submoduleFilter) {
    this.submoduleFilter = submoduleFilter;
  }

  public void debugUpdateSendMoreLines() {
    this.moreLines = true;
  }

  public void debugToggleIsLive() {
    this.isLive = !this.isLive;
  }

  public void telemetrySendCommand(String command) {
    try {
      OutputStream os = telemetryClient.getOutputStream();
      OutputStreamWriter osw = new OutputStreamWriter(os);
      BufferedWriter bw = new BufferedWriter(osw);
      StringBuilder str = new StringBuilder();
      str.append(command.length());
      while (str.toString().getBytes().length < 8) {
        str.append(' ');
      }
      bw.write(str.toString());
      bw.write(command);
      bw.flush();
      System.out.println("Sent \"" + command + "\" to client");
    } catch (IOException e) {
      System.out.println("Error sending message to telemetryClient");
    }
  }

  private class TelemetryMessageReader implements Runnable {
    @Override
    public void run() {
      System.out.println("Started reading telemetry");
      InputStream is;
      try {
        is = telemetryClient.getInputStream();
      } catch (IOException e) {
        System.out.println("IO Exception: " + e);
        throw new RuntimeException("Failed getting input stream");
      }
      InputStreamReader isr = new InputStreamReader(is);
      BufferedReader br = new BufferedReader(isr);
      while (true) {
        try {
          telemetryData = new JSONObject(br.readLine());
        } catch (IOException e) {
          System.out.println("IO Exception: " + e);
          throw new RuntimeException("Failed getting input stream");
        } catch (NullPointerException e) {
          System.out.println("telemetryClient probably disconnected");
          telemetryConnected = false;
          break;
        }
      }
    }
  }

  private static ServerSocket getServerSocket(int portNum) {
    try {
      return new ServerSocket(portNum);
    } catch (IOException e) {
      throw new RuntimeException("Failed to get new server socket");
    }
  }

  private static Socket getTelemetryClient(ServerSocket lstn) {
    try {
      return lstn.accept();
    } catch (IOException e) {
      throw new RuntimeException("Failed to get new client socket");
    }
  }

  private static void closeClient(Socket clientSocket) {
    try {
      clientSocket.close();
    } catch (IOException e) {
      throw new RuntimeException("Error closing client socket");
    }
  }

  private static void closeServer(ServerSocket serverSocket) {
    try {
      serverSocket.close();
    } catch (IOException e) {
      throw new RuntimeException("Error closing server socket");
    }
  }

  public String getTelemetryData() {
    if (telemetryData == null) {
      return null;
    }
    return telemetryData.toString();
  }

  public boolean isTelemetryConnected() {
    return telemetryConnected;
  }

  public String getTerminalOutput() {
    if (terminalOutput.isEmpty()) {
      return null;
    }
    
    JSONArray newTerminalOutput = new JSONArray();
    for (int i = 0; i < terminalOutput.length(); i++) {
      JSONObject lineJson = terminalOutput.getJSONObject(i);
      String cur_line = lineJson.toString();
      
      String log_type, submodule, debug_output;
      boolean check1 = true, check2 = true, check3 = true;

      try {
        log_type = lineJson.getString("log_type");
        
        if (!logTypes.contains(log_type)) {
          logTypes.add(log_type);
        }

        check1 = (logTypeFilter == null || log_type.contains(logTypeFilter));
      } catch (Exception e) {
        // Don't want to flood stderr
        // System.err.println("log_type field not found in terminal output line");
      }
      try {
        submodule = lineJson.getString("submodule");
        
        if (!submoduleTypes.contains(submodule)) {
          submoduleTypes.add(submodule);
        }

        check2 = (submoduleFilter == null || submodule.contains(submoduleFilter));
      } catch (Exception e) {
        // System.err.println("submodule field not found in terminal output line");
      }
      try {
        debug_output = lineJson.getString("line").split("]: ")[1].toLowerCase();
        check3 = (searchPhrase == null || debug_output.contains(searchPhrase));
      } catch (Exception e) {
        // System.err.println("debug_output field not found in terminal output line");
      }

      if (check1 && check2 && check3) {
        newTerminalOutput.put(terminalOutput.getJSONObject(i));
      }
    }

    JSONObject ret = new JSONObject();
    if (newTerminalOutput.isEmpty()) {
      ret.put("terminalOutput", JSONObject.NULL);  
    } else {
      JSONArray returnLines = new JSONArray();

      if (isLive) {
        curEnd = newTerminalOutput.length() - 1;
        curStart = Math.max(0, curEnd - 100);
      }

      if (moreLines) {
        curStart = Math.max(0, curStart - 100);
        moreLines = false;
      }

      for (int i = curStart; i <= curEnd; i++) {
        returnLines.put(newTerminalOutput.getJSONObject(i));
      }
      ret.put("terminalOutput", returnLines.toString());
    }

    ret.put("logTypes", logTypes);
    ret.put("submoduleTypes", submoduleTypes);
    ret.put("curStart", curStart);
    
    return ret.toString();
  }
}