import React from "react";
import "./Sidebar.css";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import {
  faFileImport,
  faSave,
  faPlus
} from "@fortawesome/free-solid-svg-icons";
import ReactTooltip from "react-tooltip";

const TOOLTIP_DELAY = 250;
const TOOLTIP_TEXT_COLOR = "#8f8f8f";

const Input = () => <input id="fileButton" type="file" hidden />;

const Button = props =>
  props.enabled ? (
    <div
      className="sidebar-icon"
      onClick={props.onClick}
      data-tip={props.tooltip}
    >
      {props.input ? <Input></Input> : <div />}
      <ReactTooltip
        effect="solid"
        delayShow={TOOLTIP_DELAY}
        textColor={TOOLTIP_TEXT_COLOR}
        place="left"
      />
      <FontAwesomeIcon icon={props.icon} />
    </div>
  ) : (
    <div />
  );

Button.defaultProps = { input: false, enabled: true };

export default function Sidebar(props) {
  if (props.isInactive) {
    return null;
  }
  return (
    <div id="sidebar-container">
      <Button
        onClick={props.handleUploadClick}
        tooltip="Upload config"
        input={true}
        icon={faFileImport}
      ></Button>
      <Button
        onClick={props.handleSaveClick}
        tooltip="Save config"
        icon={faSave}
      ></Button>
    </div>
  );
}
