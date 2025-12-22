import React, { useState } from 'react';
import './Chat.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => setIsOpen(!isOpen);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString()
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('https://mahnoor-sabir-book.hf.space/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: inputValue })
      });
      const data = await response.json();

      const botMessage = data.answer
        ? {
            text: data.answer,
            sender: 'bot',
            timestamp: new Date().toLocaleTimeString(),
            sources: data.sources || [],
            confidence: data.confidence
          }
        : {
            text: data.error || 'Sorry, I could not process your request.',
            sender: 'bot',
            timestamp: new Date().toLocaleTimeString()
          };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      setMessages(prev => [
        ...prev,
        {
          text: 'Error connecting to the server. Please try again later.',
          sender: 'bot',
          timestamp: new Date().toLocaleTimeString()
        }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className={`chat-widget ${isOpen ? 'chat-open' : 'chat-closed'}`}>
      <button className="chat-toggle" onClick={toggleChat}>
        {isOpen ? (
          <span className="chat-close"><img
            className="chat-close"
            src="img/drop.jpeg"
            alt="Chat Icon"
          /></span>
        ) : (
          <img
            className="chat-icon"
            src="img/chat.jpeg"
            alt="Chat Icon"
          />
        )}
      </button>

      {isOpen && (
        <div className="chat-container">
          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your AI assistant. How can I help you today?</p>
              </div>
            ) : (
              messages.map((message, index) => (
                <div key={index} className={`message ${message.sender}-message`}>
                  <div className="message-content">
                    <p>{message.text}</p>
                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <small>Sources: {message.sources.slice(0, 2).join(', ')}</small>
                      </div>
                    )}
                    {message.confidence && (
                      <div className="message-confidence">
                        <small>Confidence: {message.confidence}</small>
                      </div>
                    )}
                  </div>
                  <span className="message-time">{message.timestamp}</span>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <p>Thinking...</p>
                </div>
                <span className="message-time">{new Date().toLocaleTimeString()}</span>
              </div>
            )}
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              rows="2"
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="send-button"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
