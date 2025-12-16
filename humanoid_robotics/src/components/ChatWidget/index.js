import React, { useState, useCallback, useImperativeHandle, forwardRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import context
import styles from './styles.module.css';
import clsx from 'clsx';

const BACKEND_URL = "http://localhost:8000"; // Assuming FastAPI runs on 8000

const ChatWidget = forwardRef((props, ref) => { // Use forwardRef
  const { siteConfig } = useDocusaurusContext(); // Get site config
  const baseUrl = siteConfig.baseUrl; // Get the base URL

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]); // { text: string, sender: 'user' | 'bot' | 'system', sourceDocuments?: [] }
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Expose handleSendMessage via ref
  useImperativeHandle(ref, () => ({
    sendMessageFromExternal: (query, selectedText) => handleSendMessage(query, selectedText, true) // Pass true to indicate external call
  }));

  const handleToggleChat = () => {
    setIsOpen(!isOpen);
    setError(null);
    setInputValue('');
  };

  const handleSendMessage = useCallback(async (query, selectedText = null, isExternalCall = false) => {
    if (!query) return;

    const userMessageText = selectedText
      ? `> ${selectedText.split('\\n').join('\\n> ')}\\n\\n${query}`
      : query;

    const userMessage = { text: userMessageText, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    if (!isExternalCall) { // Only clear input if not from external call
        setInputValue('');
    }
    setError(null);
    setIsLoading(true);

    try {
        const response = await fetch(`${BACKEND_URL}/chat`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ query: query, selected_text: selectedText }),
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        const botResponse = {
            text: data.answer,
            sender: 'bot',
            sourceDocuments: data.source_documents
        };
        setMessages((prevMessages) => [...prevMessages, botResponse]);

    } catch (err) {
        console.error("API error:", err);
        setError(`Failed to get response: ${err.message}. Ensure backend is running and API keys are set.`);
        setMessages((prevMessages) => [...prevMessages, { text: "I'm sorry, I couldn't process that. Please try again.", sender: 'bot' }]);
    } finally {
        setIsLoading(false);
        setIsOpen(true); // Open chat window after sending from external
    }
  }, []); // Empty dependency array for useCallback

  // If chat is opened externally (e.g., from select-to-ask), pre-fill input with the query
  useEffect(() => {
    if (props.externalQuery && !isOpen) {
      setInputValue(props.externalQuery);
      setIsOpen(true);
    }
  }, [props.externalQuery, isOpen]);


  return (
    <div className={styles.chatContainer}>
      <button onClick={handleToggleChat} className={styles.toggleButton}>
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.messagesDisplay}>
            {messages.length === 0 && !isLoading && !error && (
                <div className={styles.welcomeMessage}>
                    Hi there! Ask me anything about the book.
                </div>
            )}
            {messages.map((msg, index) => (
              <div key={index} className={clsx(styles.message, styles[msg.sender])}>
                {msg.text}
                {msg.sourceDocuments && msg.sourceDocuments.length > 0 && (
                    <div className={styles.sourceDocuments}>
                        <strong>Sources:</strong>
                        <ul>
                            {msg.sourceDocuments.map((source, srcIdx) => {
                                // Manually construct the URL with the base URL
                                const docUrl = `${baseUrl}docs/${source.source.replace(/\\/g, '/')}`;
                                return (
                                    <li key={srcIdx}>
                                        <a href={docUrl} target="_blank" rel="noopener noreferrer">
                                            {source.source} (Chunk {source.chunk_id})
                                        </a>
                                    </li>
                                );
                            })}
                        </ul>
                    </div>
                )}
              </div>
            ))}
            {isLoading && (
                <div className={clsx(styles.message, styles.bot)}>
                    Typing...
                </div>
            )}
            {error && (
                <div className={clsx(styles.message, styles.systemError)}>
                    Error: {error}
                </div>
            )}
          </div>
          <div className={styles.inputArea}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleSendMessage(inputValue)}
              placeholder="Type your question..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button onClick={() => handleSendMessage(inputValue)} className={styles.sendButton} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
});

export default ChatWidget;