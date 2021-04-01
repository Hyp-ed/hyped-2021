package server;

import javax.annotation.*;
import org.json.*;
import org.springframework.web.bind.annotation.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.handler.annotation.SendTo;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.scheduling.TaskScheduler;

@RestController
public class Controller {
  @Autowired
  private Server server;

  @Autowired
  private SimpMessagingTemplate template;

  @Autowired
  private TaskScheduler scheduler;

  private final int TELEMETRY_PING_INTERVAL = 100;
  private final int DEBUG_PING_INTERVAL = 1000;

  @PostConstruct
  public void initialize() {
    if (server != null) {
      Thread serverThread = new Thread(server);
      serverThread.start();
      System.out.println("Server started");
      startPingingData();
    }
  }

  public void startPingingData() {
    Thread checkToScheduleThread = new Thread(
      new Runnable() {

        @Override
        public void run() {
          scheduler.scheduleAtFixedRate(() -> pingTelemetryConnection(), TELEMETRY_PING_INTERVAL);
          scheduler.scheduleAtFixedRate(() -> pingTelemetryData(), TELEMETRY_PING_INTERVAL);
          scheduler.scheduleAtFixedRate(() -> pingTerminalOutput(), DEBUG_PING_INTERVAL);
          scheduler.scheduleAtFixedRate(() -> pingDebugConnection(), DEBUG_PING_INTERVAL);
          scheduler.scheduleAtFixedRate(() -> pingDebugStatus(), DEBUG_PING_INTERVAL);
          scheduler.scheduleAtFixedRate(() -> pingDebugData(), DEBUG_PING_INTERVAL);
          return; // end thread
        }
      }
    );

    checkToScheduleThread.start();
  }

  @MessageMapping("/send/debug/connection")
  public void debugConnect(String serverName) {
    // TODO(Steven): implement SSH connection
  }

  @MessageMapping("/send/debug/compile")
  public void debugCompile(String flagsString) {
    server.debugCompile();
  }

  @MessageMapping("/send/debug/run")
  public void debugRun(String flagsString) {
    JSONArray data = new JSONArray(flagsString);
    server.debugRun(data);
  }

  @MessageMapping("/send/debug/kill")
  public void debugKill() {
    server.debugKill();
  }

  @MessageMapping("/send/debug/search")
  public void debugUpdateSearchPhrase(String jsonStr) {
    String searchPhrase = null;
    if (jsonStr != null) {
      try {
        JSONObject search = new JSONObject(jsonStr);
        
        try {
          searchPhrase = search.getString("searchPhrase");
        } catch (Exception e) {
          System.out.println("Key \"searchPhrase\" does not exist");
        }
      } catch (Exception e) {
        System.out.println("Failed parsing jsonStr into JSONObject");
      }
    }
    server.debugUpdateSearchPhrase(searchPhrase);
  } 

  @MessageMapping("/send/debug/logType")
  public void debugUpdateLogType(String jsonStr) {
    String logType = null;
    if (jsonStr != null) {
      try {
        JSONObject obj = new JSONObject(jsonStr);
        
        try {
          logType = obj.getString("logType");
        } catch (Exception e) {
          System.out.println("Key \"logType\" does not exist");
        }
      } catch (Exception e) {
        System.out.println("Failed parsing jsonStr into JSONObject");
      }
    }
    server.debugUpdateLogTypeFilter(logType);
  }

  @MessageMapping("/send/debug/submodule")
  public void debugUpdateSubmodule(String jsonStr) {
    String submodule = null;
    if (jsonStr != null) {
      try {
        JSONObject obj = new JSONObject(jsonStr);
        
        try {
          submodule = obj.getString("submodule");
        } catch (Exception e) {
          System.out.println("Key \"submodule\" does not exist");
        }
      } catch (Exception e) {
        System.out.println("Failed parsing jsonStr into JSONObject");
      }
    }
    server.debugUpdateSubmoduleFilter(submodule);
  }

  @MessageMapping("/send/debug/moreLines")
  public void debugUpdateSendMoreLines() {
    server.debugUpdateSendMoreLines();
  }

  @MessageMapping("/send/debug/isLive")
  public void debugToggleIsLive() {
    server.debugToggleIsLive();
  }

  @MessageMapping("/send/telemetry/command")
  @SendTo("/topic/send/telemetry/command/status")
  public String sendTelemetryCommand(String command) {
    try {
      if (server.isTelemetryConnected()) {
        server.telemetrySendCommand(command);
        return getResponseMessage("success", "successfully sent command " + command);
      }
    } catch (JSONException e) {
      return getResponseMessage("error", "poorly formed json attempted to be sent to server (probs entered nothing in run_length box)");
    }

    return getResponseMessage("error", "could not send message");
  }

  public void pingTelemetryConnection() {
    if (!server.isTelemetryConnected()) {
      template.convertAndSend("/topic/telemetry/connection", "DISCONNECTED");
    } else {
      template.convertAndSend("/topic/telemetry/connection", "CONNECTED");
    }
  }

  public void pingTelemetryData() {
    if (server.getTelemetryData() != null) {
      template.convertAndSend(
        "/topic/telemetry/data",
        server.getTelemetryData()
      );
    }
  }

  public void pingDebugStatus() {
    template.convertAndSend("/topic/debug/status", server.getDebugStatus());
  }

  public void pingDebugConnection() {
    // TODO(Steven): implement debugConnection
  }

  public void pingTerminalOutput() {
    if (server.getTerminalOutput() != null) {
      template.convertAndSend(
        "/topic/debug/output",
        server.getTerminalOutput()
      );
    }
  }

  public void pingDebugData() {
    template.convertAndSend("/topic/debug/data", server.getDebugData());
  }

  public String getResponseMessage(String status, String message) {
    JSONObject response = new JSONObject();
    response.put("status", status);
    response.put("message", message);
    return response.toString();
  }
}
