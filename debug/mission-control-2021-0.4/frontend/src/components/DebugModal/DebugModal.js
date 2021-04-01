import React, { useEffect } from "react";
import Modal from 'react-modal';
import "./DebugModal.css";
import Button from "../Button/Button";
import {
  faRedo,
} from "@fortawesome/free-solid-svg-icons";
import SimpleBar from "simplebar-react";
import "simplebar/dist/simplebar.min.css";

export default React.memo(props => {
  Modal.setAppElement('#root');

  const handleCompileClick = () => {
    props.setDebugModalOpen(false);
    props.stompClient.send("/app/send/debug/compile", {}, "COMPILE");
    props.setDebugStatus("COMPILING");
  };


  useEffect(() => {
    if (props.debugStatus === "RETRY") {
      props.setDebugModalOpen(true);
    }
  }, [props.debugStatus]);
  
  const closeModal = () => {
    props.setDebugModalOpen(false);
  }
  
  const getErrorMessage = () => {
    return props.debugErrorMessage.split("\n").map(msg => {
      if (msg.length > 0) {
        return <pre className="errormsg">{msg}</pre>;
      }
      else {
        return null;
      }
    })
  }

  // TODO: fittext
  return (
    <Modal
      isOpen={props.isDebugModalOpen}
      onRequestClose={closeModal}
      className="modal-run"
      overlayClassName="modal-run-overlay"
    >
      <div className="modal-dialog">
        <SimpleBar className="debug-wrapper centered container" forceVisible="y" autoHide={false}>
          <div className="debug-group">
            <label>Compilation failed</label>
            <div className="errormsgs">{getErrorMessage()}</div>
          </div>
          <div className="debug-wrapper-buttons">
            <Button
              caption="RETRY"
              handleClick={handleCompileClick}
              backgroundColor="button-red"
              icon={faRedo}
            ></Button>
          </div>
        </SimpleBar>
      </div>
    </Modal>
  );
})
