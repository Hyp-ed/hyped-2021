import React, { useState } from "react";
import "./DataList.css";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCaretUp, faCaretDown } from "@fortawesome/free-solid-svg-icons";
import { Collapse } from "react-collapse";

export default function DataList(props) {
  const [isOpen, setIsOpen] = useState(true);
  return (
    <div>
      <div className="data-list-header" onClick={() => setIsOpen(!isOpen)}>
        <div className="data-list-header-content">
          <div className="data-list-header-content-title">{props.title}</div>
          <FontAwesomeIcon
            className="data-list-header-content-caret"
            icon={isOpen ? faCaretDown : faCaretUp }
          ></FontAwesomeIcon>
        </div>
      </div>
      <Collapse theme={{collapse: "data-list-body-collapse", content: "data-list-body-content"}} isOpened={isOpen}>
        {props.value}
      </Collapse>
    </div>
  );
};
