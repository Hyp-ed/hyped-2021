import React, { useEffect, useState } from "react";
import "./DataPointSelector.css";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTimes } from "@fortawesome/free-solid-svg-icons";
import { getAllPaths } from "../../DataTools";
import SimpleBar from "simplebar-react";
import "simplebar/dist/simplebar.min.css";

export default function DataPointSelector(props) {
  const [paths, setPaths] = useState();

  useEffect(() => {
    setPaths(getAllPaths(props.telemetryData));
  }, [props.telemetryData]);

  const getPathList = () => {
    // const paths = ;
    return paths
      .filter(path => path !== undefined)
      .map((path, i) => (
        <div
          key={i}
          onClick={() => props.onDataPointClicked(path.path)}
          className={[
            "dataPoint",
            props.isSelected(path.path) ? "dataPoint--selected" : ""
          ].join(" ")}
          // onClick={}
        >
          {path.caption}
        </div>
      ));
  };

  //<div id="list">{getPathList()}</div>

  return props.visible ? (
    <div id="list-container">
      <FontAwesomeIcon
        id="list-close-button"
        onClick={props.onCloseClicked}
        icon={faTimes}
      />

      <SimpleBar className="list-bar" forceVisible="x" autoHide={false} >
        <div id="list">{getPathList()}</div>
      </SimpleBar>
    </div>
  ) : (
    <div />
  );
}
