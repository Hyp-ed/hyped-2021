import React from "react";
import "./DataContainer.css";
import DataRow from "../DataRow/DataRow";
import DataList from "../DataList/DataList";
import SimpleBar from "simplebar-react";
import "simplebar/dist/simplebar.min.css";

export default function DataContainer(props) {
  var dataList = {};

  const combine = (title, data) => {
    for (var list of Object.values(data)) {
      if (Array.isArray(list.value)) {
        combine(title.concat([list.name]), list.value);
      }
      else {
        if (title.join(" > ") in dataList === false) {
          dataList[title.join(" > ")] = [];
        }
        dataList[title.join(" > ")].push(list);
      }
    };
  }

  const getListItems = (list) => {
    return list.map((data, i) => {
      return <DataRow data={data} key={data.name}></DataRow>;
    });
  }

  const getFormattedLists = () => {
    var response = []
    for (var title in dataList) {
      response.push(
        <DataList
          title={title}
          value={getListItems(dataList[title])}
          key={title}
        ></DataList>
      )
    }
    return response;
  }

  if (props.telemetryData !== null) {
    combine([], props.telemetryData.additional_data);
    
    return (
      <SimpleBar className="data-container" forceVisible="y" autoHide={false}>
        {getFormattedLists()}
      </SimpleBar>
    );
  }
  else {
    return (<div className="data-container container"></div>);
  }  
};
