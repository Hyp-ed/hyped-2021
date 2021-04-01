import React from "react";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import "./Button.css";

export default function Button(props) {
  const getClassNames = backgroundColor => {
    var classes = ["button"];

    if (props.disabled) {
      classes.push("disabled");
    }
    classes.push(backgroundColor);

    return classes.join(" ");
  };

  return (
    <button
      className={getClassNames(props.backgroundColor)}
      onClick={props.handleClick}
    >
      <FontAwesomeIcon
        className="button-icon"
        icon={props.icon}
        spin={props.spin}
      />
      {props.caption}
    </button>
  );
}
