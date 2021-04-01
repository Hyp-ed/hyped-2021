import {
  faSpinner,
  faCogs,
  faPlay,
  faRedo,
  faCheck
} from "@fortawesome/free-solid-svg-icons";
import React, { useState, useEffect } from "react";
import Button from "../Button/Button";

export default function ButtonContainerDebug(props) {
  const buttons = {
    loading: {
      caption: "LOADING",
      icon: faSpinner,
      spin: true,
      backgroundColor: "button-blue",
      disabled: true
    },
    compile: {
      caption: "COMPILE",
      icon: faCogs,
      backgroundColor: "button-blue",
      command: "COMPILE",
      disabled: false
    },
    compiling: {
      caption: "COMPILING",
      icon: faSpinner,
      spin: true,
      backgroundColor: "button-blue",
      disabled: true
    },
    retry: {
      caption: "RETRY",
      icon: faRedo,
      backgroundColor: "button-red",
      command: "RETRY",
      disabled: false
    },
    compiled: {
      caption: "RECOMPILE",
      icon: faCheck,
      backgroundColor: "button-green",
      command: "RECOMPILE",
      disabled: false
    },
    run: {
      caption: "RUN",
      icon: faPlay,
      backgroundColor: "button-blue",
      command: "RUN",
      disabled: false
    },
  };

  const handleClick = (command, disabled, debug = false) => {
    if (disabled || !props.stompClient) {
      return;
    }

    switch (command) {
      case "RUN":
        props.setModalOpen(true);
        break;
      case "COMPILE":
      case "RECOMPILE":
      case "RETRY":
        props.stompClient.send("/app/send/debug/compile", {}, command);
        props.setDebugStatus("COMPILING");
        break;
      default:
        break;
    }
  };

  const getButton = (button) => {
    return (
      <Button
        caption={button.caption}
        icon={button.icon}
        spin={button.spin}
        handleClick={() => {
          handleClick(button.command, button.disabled);
        }}
        backgroundColor={button.backgroundColor}
        disabled={button.disabled}
        key={button.caption}
      ></Button>
    );
  };

  const getDebugStatus = () => {
    switch(props.debugStatus) {
      case "COMPILE":
        return buttons.compile;
      case "COMPILING":
        return buttons.compiling;
      case "RECOMPILE":
        return buttons.compiled;
      case "RETRY":
        return buttons.retry;
      default:
        return buttons.loading;
    }
  }

  const getButtons = (isCompiled = false) => {
    let button = getDebugStatus();

    if (isCompiled) {
      return [getButton(button), getButton(buttons.run)];
    }
    else {
      return [getButton(button)];
    }
  }

  return (
    <div className="button-container">
      {getButtons(props.debugData.isCompiled)}
    </div>
  );
};
