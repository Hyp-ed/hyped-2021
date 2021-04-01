import {
  faRuler,
  faExclamationTriangle,
  faSpinner,
  faForward,
  faPowerOff,
  faLockOpen,
  faLock,
} from "@fortawesome/free-solid-svg-icons";
import React, { useState, useEffect } from "react";
import Button from "../Button/Button";

export default function ButtonContainerTelemetry(props) {
  const [isMainDisabled, setMainDisabled] = useState(false);
  
  const buttons = {
    calibrate: {
      caption: "CALIBRATE",
      icon: faRuler,
      backgroundColor: "button-blue-pulse",
      command: "CALIBRATE"
    },
    calibrating: {
      caption: "CALIBRATING",
      icon: faSpinner,
      spin: true,
      backgroundColor: "button-blue"
    },
    launch: {
      caption: "LAUNCH",
      icon: faForward,
      backgroundColor: "button-green-pulse",
      command: "LAUNCH"
    },
    shutdown: {
      caption: "SHUTDOWN",
      icon: faPowerOff,
      backgroundColor: "button-red",
      command: "SHUTDOWN"
    },
    abort: {
      caption: "ABORT",
      icon: faExclamationTriangle,
      backgroundColor: "button-red",
      command: "STOP"
    },
    brakes_retract: {
      caption: "RETRACT",
      icon: faLockOpen,
      backgroundColor: "button-blue",
      command: "NOMINAL_RETRACT"
    },
    brakes_engage: {
      caption: "ENGAGE",
      icon: faLock,
      backgroundColor: "button-blue",
      command: "NOMINAL_BRAKING"
    }
  };

  const handleClick = (command, disabled) => {
    if (disabled || !props.stompClient) {
      return;
    }
    if (command !== "NOMINAL_RETRACT" && command != "NOMINAL_BRAKING") {
      setMainDisabled(true);
    }

    props.stompClient.send("/app/send/telemetry/command", {}, command);
    console.log("Sent command: " + command);
  };

  useEffect(() => {
    setMainDisabled(false);
  }, [props.state]);

  const getButton = (button, disabled = false) => {
    return (
      <Button
        caption={button.caption}
        icon={button.icon}
        spin={button.spin}
        handleClick={() => {
          handleClick(button.command, isMainDisabled);
        }}
        backgroundColor={button.backgroundColor}
        disabled={disabled}
        key={button.caption}
      ></Button>
    );
  };

  const getMainButton = () => {
    switch (props.state) {
      case "IDLE":
        return getButton(buttons.calibrate, isMainDisabled);
      case "CALIBRATING":
        return getButton(buttons.calibrating, true);
      case "READY":
        return getButton(buttons.launch, isMainDisabled);
      case "ACCELERATING":
      case "EMERGENCY_BRAKING":
      case "NOMINAL_BRAKING":
        return null;
      case "FINISHED":
      case "FAILURE_STOPPED":
      default:
        return getButton(buttons.shutdown, isMainDisabled);
    }
  };

  const getAbortButton = () => {
    switch (props.state) {
      case "IDLE":
      case "CALIBRATING":
      case "READY":
      case "ACCELERATING":
      case "NOMINAL_BRAKING":
        return getButton(buttons.abort, isMainDisabled);
      case "EMERGENCY_BRAKING":
        return getButton(buttons.abort, true);
      case "FINISHED":
      case "FAILURE_STOPPED":
      default:
        return null;
    }
  };

  const getBrakesButton = () => {
    switch (props.state) {
      case "IDLE":
      case "FINISHED":
        return ([
            getButton(buttons.brakes_retract, isMainDisabled),
            getButton(buttons.brakes_engage, isMainDisabled)]
        );
      default:
        return null;
    }
  };

  return (
    <div className="button-container">
      {getAbortButton()}
      {getBrakesButton()}
      {getMainButton()}
    </div>
  );
};
