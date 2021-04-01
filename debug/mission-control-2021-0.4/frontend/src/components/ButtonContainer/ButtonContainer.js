import "./ButtonContainer.css";
import React from "react";
import ButtonContainerTelemetry from "./ButtonContainerTelemetry";
import ButtonContainerDebug from "./ButtonContainerDebug";

export default function ButtonContainer(props) {
  if (props.telemetryConnection) {
    return (<ButtonContainerTelemetry {...props}></ButtonContainerTelemetry>);
  }
  else {
    return (<ButtonContainerDebug {...props}></ButtonContainerDebug>);
  }
};
