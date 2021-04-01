import React, { useState, useEffect, useRef, useCallback } from "react";
import "./Terminal.css";
import Button from "../Button/Button";
import { faArrowDown, faCaretUp, faCaretDown, faTimesCircle} from "@fortawesome/free-solid-svg-icons";
import SimpleBar from "simplebar-react";
import "simplebar/dist/simplebar.min.css";

function DropdownItem(props) {
  return (
    <a href="#" className="menu-item" onClick={props.clickEffect}>
      {props.children}
    </a>
  )
}

export default function Terminal(props) {
  const scrollableNodeRef = React.createRef();
  const previousOutput = usePrevious(props.terminalOutput);
  const [loading, setLoading] = useState(false);
  const [isLive, setIsLive] = useState(true);
  const [scrollCnt, setScrollCnt] = useState(0);
  const [open, setOpen] = useState("");

  const topObserver = useRef()
  const firstLineRef = useCallback(node => {
    if (loading) {
      return
    }
    if (topObserver.current) {
      topObserver.current.disconnect()
    }
    topObserver.current = new IntersectionObserver(entries => {
      if (entries[0].isIntersecting && props.curStart !== 0 && !isLive) {
        setLoading(true);
        setScrollCnt(Math.min(100, props.curStart - 0));
        props.stompClient.send("/app/send/debug/moreLines", {}, {});
      }
    })
    if (node) {
      topObserver.current.observe(node)
    }
  }, [loading, props.curStart, isLive, props.stompClient])
  
  const botObserver = useRef()
  const lastLineRef = useCallback(node => {
    const updateIsLive = (value) => {
      if (value === isLive) {
        return;
      }
      setIsLive(!isLive);
      props.stompClient.send("/app/send/debug/isLive", {}, {});
    }
    if (loading) {
      return
    }
    if (botObserver.current) {
      botObserver.current.disconnect()
    }
    botObserver.current = new IntersectionObserver(entries => {
      if (!entries[0].isIntersecting) {
        updateIsLive(false);
      } else {
        updateIsLive(true);
      }
    })
    if (node) {
      botObserver.current.observe(node)
    }
  }, [loading, isLive, props.stompClient])

  function usePrevious(value) {
    const ref = useRef();
    useEffect(() => {
      ref.current = value;
    });
    return ref.current;
  }

  useEffect(() => {
    if (loading) {
      var position = scrollableNodeRef.current.scrollHeight / props.terminalOutput.length * scrollCnt;
      scrollableNodeRef.current.scrollTo(0, position);
      setLoading(false);
    }
  }, [loading, scrollableNodeRef, props.terminalOutput.length, scrollCnt]);

  // updates terminal output in the pre
  useEffect(() => {
    if (isLive && JSON.stringify(previousOutput) !== JSON.stringify(props.terminalOutput) && scrollableNodeRef.current !== null) {
      scrollableNodeRef.current.scrollTo(0, scrollableNodeRef.current.scrollHeight);
    }
    return function cleanup() {
      // TODO: we can cleanup the terminal here when the terminal is inactive
      //  or when too much is in the pre
      if (props.isInactive) console.log("terminal off");
    };
  }, [props.terminalOutput, isLive, previousOutput, props.isInactive, scrollableNodeRef]); //useEffect only called when terminal output changes


  if (props.isInactive) {
    return null;
  }

  let terminalOut = 
    props.terminalOutput && 
    props.terminalOutput.length > 0 && 
    props.terminalOutput.map((log, index) => {
      if (index === 0) {
        return <div ref={firstLineRef} key={index}>{log.line}</div>
      } else if (index === props.terminalOutput.length - 1) {
        return <div ref={lastLineRef} key={index}>{log.line}</div>
      } else {
        return <div key={index}>{log.line}</div>
      }
    })

  const handleKillClick = () => {
    props.stompClient.send("/app/send/debug/kill", {}, {});
  }

  const scrollToEnd = () => {
    if (scrollableNodeRef.current != null) {
      scrollableNodeRef.current.scrollTo(0, scrollableNodeRef.current.scrollHeight);
    }
  }

  const handleSearch = (event) => {
    var myObj = {"searchPhrase": event.target.value};
    props.stompClient.send("/app/send/debug/search", {}, JSON.stringify(myObj));
    scrollToEnd();
  }

  const filterLogType = (value) => {
    var myObj = {"logType": value};
    props.stompClient.send("/app/send/debug/logType", {}, JSON.stringify(myObj));
    scrollToEnd();
    setOpen("");
  }

  const filterSubmodule = (value) => {
    var myObj = {"submodule": value};
    props.stompClient.send("/app/send/debug/submodule", {}, JSON.stringify(myObj));
    scrollToEnd();
    setOpen("");
  }

  let logTypeOptions = 
    props.logTypes &&
    props.logTypes.length > 0 &&
    props.logTypes.map((logType, index) => {
      return <DropdownItem clickEffect={() => filterLogType(logType)}>{logType}</DropdownItem>
    })

  let submoduleOptions = 
    props.submoduleTypes &&
    props.submoduleTypes.length > 0 &&
    props.submoduleTypes.map((submoduleType, index) => {
      return <DropdownItem clickEffect={() => filterSubmodule(submoduleType)}>{submoduleType}</DropdownItem>
    })

  if (props.terminalOutput !== "") {
    return (
      <div id="terminal-container" className="container">
        <SimpleBar className="terminal-content" forceVisible="y" autoHide={false} scrollableNodeProps={{ ref: scrollableNodeRef }}>
          <pre id="terminal_pre">{terminalOut}</pre>
        </SimpleBar>
        <div className="footer filtering">
          <Button
            caption="Kill Process"
            icon={faTimesCircle}
            handleClick={handleKillClick}
          ></Button>
          <div className="dropdown-group">
            <Button 
            caption="Log Type"
            icon={open === "log" ? faCaretUp : faCaretDown}
            handleClick={() => {
              if (open === "log") {
                setOpen("");
              } else {
                setOpen("log");
              }
            }}
            ></Button>
            {open === "log" && 
              <div className="dropdown">
                {logTypeOptions}
              </div>
            }
          </div>
          <div className="dropdown-group">
            <Button 
              caption="Submodule"
              icon={open === "sub" ? faCaretUp : faCaretDown}
              handleClick={() => {
                if (open === "sub") {
                  setOpen("");
                } else {
                  setOpen("sub");
                }
              }}
            ></Button>
            {open === "sub" && 
                <div className="dropdown">
                  {submoduleOptions}
                </div>
              }
          </div>
        </div>
        <div className="footer other">
          <input
            type="text"
            placeholder="Search..." 
            name="search"
            onChange={handleSearch}
          ></input>
        </div>
        {!isLive && <div className="toEndWrapper">
          <Button
            caption=""
            icon={faArrowDown}
            width="38%"
            handleClick={scrollToEnd}
          ></Button>
        </div>}
      </div>
    );
  }
  else {
    return (<div id="terminal-container" className="container"></div>);
  }
}

Terminal.defaultProps = {
  content: "The content prop of this component has not been set",
  flags: [],
  debug_level: "0"
};