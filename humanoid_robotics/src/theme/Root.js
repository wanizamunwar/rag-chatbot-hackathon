import React, { useRef, useState, useEffect } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import SelectToAskButton from '@site/src/components/SelectToAskButton';

function Root({ children }) {
  const chatWidgetRef = useRef(null);
  const [selection, setSelection] = useState({ text: null, position: null });

  const handleMouseUp = () => {
    const selectedText = window.getSelection().toString().trim();

    if (selectedText) {
      const selectionRange = window.getSelection().getRangeAt(0);
      const rect = selectionRange.getBoundingClientRect();
      
      setSelection({
        text: selectedText,
        position: {
          x: rect.left + window.scrollX + (rect.width / 2) - 60, // Center the button
          y: rect.top + window.scrollY - 45, // Position above the selection
        },
      });
    } else {
      setSelection({ text: null, position: null });
    }
  };

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleSelectToAsk = () => {
    if (selection.text && chatWidgetRef.current) {
      const query = `What does "${selection.text}" mean?`;
      chatWidgetRef.current.sendMessageFromExternal(query, selection.text);
      setSelection({ text: null, position: null }); // Hide button after click
    }
  };

  return (
    <>
      {children}
      <SelectToAskButton position={selection.position} onClick={handleSelectToAsk} />
      <ChatWidget ref={chatWidgetRef} />
    </>
  );
}

export default Root;
