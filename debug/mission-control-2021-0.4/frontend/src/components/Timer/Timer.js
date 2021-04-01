import React, { useState, useEffect } from "react";
import "./Timer.css";

export default function Timer(props){
    
    const [time, setTime] = useState(0);

    useEffect(() => {
    if (props.endTime !== 0) {
      return;
    }
    if (props.startTime === 0) {
      return () => setTime(0);
    }
    return () => setTime(props.telemetryData.time - props.startTime);
    }, [props.startTime, props.endTime,props.telemetryData]); //sets interval once when timer state changes

    const formatTime = duration => {
    var milliseconds = parseInt(duration%1000),
      seconds = Math.floor((duration / 1000) % 60),
      minutes = Math.floor((duration / (1000 * 60)) % 60);

    minutes = minutes < 10 ? "0" + minutes : minutes;
    seconds = seconds < 10 ? "0" + seconds : seconds;

    if (milliseconds < 10) {
      milliseconds = "00" + milliseconds;
    } else if (milliseconds < 100) {
      milliseconds = "0" + milliseconds;
    }

    return  minutes + ":" + seconds + "." + milliseconds ;
  };

  if (props.telemetryData !== null) {
    return (
      <div id="timer" className="container">
        <p className="timer" id="time">{formatTime(time)}</p>
      </div>
    );
  }
  else {
    return (<div id="timer" className="container"></div>);
  }
  
}