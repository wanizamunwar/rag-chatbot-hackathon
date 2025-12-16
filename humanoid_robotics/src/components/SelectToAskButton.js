import React from 'react';
import styles from './SelectToAskButton.module.css';

const SelectToAskButton = ({ position, onClick }) => {
  if (!position) {
    return null;
  }

  return (
    <button
      className={styles.selectToAskButton}
      style={{ top: position.y, left: position.x }}
      onClick={onClick}
    >
      Ask about this
    </button>
  );
};

export default SelectToAskButton;