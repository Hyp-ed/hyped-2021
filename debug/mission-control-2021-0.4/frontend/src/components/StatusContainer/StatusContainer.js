import React from "react";
import 'status-indicator/styles.css'
import "./StatusContainer.css";

export default function StatusContainer(props) {
    const getModulesColumn = () => {
        return (
            <div className="status-modules">
                <div className="status-title">Modules</div>
                <div className="status-modules-content">
                    {getModuleIndicator("MOT")}
                    {getModuleIndicator("BAT")}
                    {getModuleIndicator("SEN")}
                    {getModuleIndicator("TEL")}
                    {getModuleIndicator("NAV")}
                    {getModuleIndicator("BRA")}
                </div>
            </div>
        );
    }

    const getStatusColumn = (title) => {
        return (
            <div className="status-sensors">
                <div className="status-title">{title}</div>
                <div className="status-sensors-content">
                    {getStatusIndicator(1)}
                    {getStatusIndicator(2)}
                    {getStatusIndicator(3)}
                    {getStatusIndicator(4)}
                </div>
            </div>
        );
    }

    const getModuleIndicator = (title) => {
        // TODO: connect to the backend
        return (<div className="status-indicator">{title}</div>);
    }

    const getStatusIndicator = (title) => {
        // TODO: connect to the backend
        return (<div className="status-indicator">{title}</div>);
    }

    // 0 - red, 1 - green, 2 - blue, 3 - orange
    const getConnectionIndicator = (status) => {
        switch(status) {
            case 0:
                return (<status-indicator negative pulse></status-indicator>);
            case 1:
                return (<status-indicator positive pulse></status-indicator>);
            case 2:
                return (<status-indicator active pulse></status-indicator>);
            case 3:
                return (<status-indicator intermediary pulse></status-indicator>);
            default:
                return null;
        }
    }

    const getTelemetryConnectionStatus = () => {
        return (<div>{getConnectionIndicator(props.telemetryConnection ? 1 : 0)}{props.telemetryConnection ? "CONNECTED" : "DISCONNECTED"}</div>);
    }

    const getDebugStatus = () => {
        // TODO: implement debug status
        return null;
    }

    const getSTMStatus = () => {
        if (!props.telemetryConnection) {
            return null;
        }
        var indicator = 0;
        switch (props.state) {
            case "EMERGENCY_BRAKING":
                indicator = 3;
                break;
            case "IDLE":
            case "CALIBRATING":
                indicator = 2;
                break;
            case "READY":
            case "ACCELERATING":
            case "NOMINAL_BRAKING":
            case "FINISHED":
            case "RUN_COMPLETE":
                indicator = 1;
                break;
            case "FAILURE_STOPPED":
            default:
                indicator = 0;
                break;
        }
        return (<div>{getConnectionIndicator(indicator)}{props.state}</div>);
    }

    return (
        <div id="status">
            <div id="connection-container">
                {getTelemetryConnectionStatus()}
                {getDebugStatus()}
                {getSTMStatus()}
            </div>
            { props.telemetryData !== null
            ? <div id="status-container" className="container">
                {/* TODO: connect to the backend */}
                <div className="status-sensors-container">
                    <div className="status-sensors-row status-sensors-row-top">
                        {getStatusColumn("Sensors 1")}
                        {getStatusColumn("Sensors 2")}
                        {getStatusColumn("Sensors 3")}
                    </div>
                    <div className="status-sensors-row status-sensors-row-bottom">
                        {getStatusColumn("Sensors 4")}
                        {getStatusColumn("Sensors 5")}
                        {getStatusColumn("Sensors 6")}
                    </div>
                </div>
                {getModulesColumn()}
            </div>
            : <div id="status-container" className="container"></div>
            }
        </div>
    );
}