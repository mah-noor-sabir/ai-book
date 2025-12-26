import React, { useState, useRef, useEffect } from 'react';
import { FiSend, FiMoreVertical, FiTrash2, FiClock, FiCopy } from 'react-icons/fi';
import './Chat.css';

const ChatWidget = () => {
  // Chat state
  const [isOpen, setIsOpen] = useState(() => JSON.parse(localStorage.getItem("chat_open_state")) || false);
  const [messages, setMessages] = useState(() => JSON.parse(localStorage.getItem("chat_messages")) || []);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [menuOpen, setMenuOpen] = useState(false);
  const [historyOpen, setHistoryOpen] = useState(false);
  const [copiedIndex, setCopiedIndex] = useState(null);

  const messagesEndRef = useRef(null);
  const menuRef = useRef(null);

  // Detect system theme
  const [isLightMode, setIsLightMode] = useState(false);
  useEffect(() => {
    const darkMode = window.matchMedia('(prefers-color-scheme: dark)').matches;
    setIsLightMode(!darkMode);
  }, []);

  // Persist chat messages and open state
  useEffect(() => localStorage.setItem("chat_messages", JSON.stringify(messages)), [messages]);
  useEffect(() => localStorage.setItem("chat_open_state", JSON.stringify(isOpen)), [isOpen]);

  // Auto-scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(() => { scrollToBottom(); }, [messages, isLoading]);

  // Toggle chat, menu, history
  const toggleChat = () => setIsOpen(!isOpen);
  const toggleMenu = () => setMenuOpen(!menuOpen);
  const toggleHistory = () => setHistoryOpen(!historyOpen);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (e) => {
      if (menuRef.current && !menuRef.current.contains(e.target)) setMenuOpen(false);
    };
    if (menuOpen) document.addEventListener('click', handleClickOutside);
    return () => document.removeEventListener('click', handleClickOutside);
  }, [menuOpen]);

  // Greetings fallback
  const greetings = {
    hi: "Hello! 👋 How can I help you with robotics today?",
    hello: "Hi there! I’m your AI Robotics Assistant.",
    hey: "Hey! Ask me anything about humanoid AI or robotics.",
    goodmorning: "Good morning! Ready to explore some AI?",
    goodnight: "Good night! Don’t forget to dream of robots 🤖."
  };

  // Backend URL
  const getBackendURL = () => 'https://mahnoor-sabir-book.hf.space/ask';

  // Send message
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userText = inputValue.trim();
    const userTextLower = userText.toLowerCase();
    const userMessage = { text: userText, sender: "user", timestamp: new Date().toISOString() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Check greetings
    for (const key in greetings) {
      if (userTextLower.includes(key)) {
        const botMessage = { text: greetings[key], sender: "bot", timestamp: new Date().toISOString() };
        setMessages(prev => [...prev, botMessage]);
        return;
      }
    }

    // Send to backend
    setIsLoading(true);
    try {
      const response = await fetch(getBackendURL(), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: userText }),
        mode: 'cors',
      });

      if (!response.ok) throw new Error(`Server error: ${response.status}`);
      const data = await response.json();

      const fallback = [
        "Hmm, I’m not sure about that, but let’s explore it together! 🤖",
        "Interesting question! I’ll learn more about it next time.",
        "Haha, I don’t know that yet, but I’m always learning!",
        "That’s a curious one! Let’s keep discovering together."
      ];

      const botMessage = data.answer && data.answer.trim() !== ""
        ? { text: data.answer, sender: "bot", timestamp: new Date().toISOString() }
        : { text: fallback[Math.floor(Math.random() * fallback.length)], sender: "bot", timestamp: new Date().toISOString() };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error("SendMessage error:", error);
      setMessages(prev => [...prev, { text: 'Error connecting to server.', sender: 'bot', timestamp: new Date().toISOString() }]);
    } finally {
      setIsLoading(false);
    }
  };

  // Enter key handler
  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Copy message
  const handleCopy = (text, index) => {
    navigator.clipboard.writeText(text);
    setCopiedIndex(index);
    setTimeout(() => setCopiedIndex(null), 2000);
  };

  // Delete chat
  const deleteChat = () => {
    setMessages([]);
    localStorage.removeItem("chat_messages");
  };

  return (
    <div className={`chat-widget ${isLightMode ? 'light-mode' : ''}`}>
      {/* Toggle Button */}
      <div className="chat-toggle-wrapper">
        <button className="chat-toggle" onClick={toggleChat}>
          <img src={isOpen ? "/img/drop.jpeg" : "/img/chat.jpeg"} alt={isOpen ? "Close Chat" : "Open Chat"} />
        </button>
      </div>

      {isOpen && (
        <div className="chat-container">
          {/* Header */}
          <div className="chat-header">
            <span>🤖 Physical AI & Humanoid Robotics Assistant</span>
            <div className="chat-menu" ref={menuRef}>
              <FiMoreVertical onClick={toggleMenu} cursor="pointer" />
              {menuOpen && (
                <div className="menu-dropdown">
                  <button onClick={toggleHistory}><FiClock /> History</button>
                  <button onClick={deleteChat}><FiTrash2 /> Delete Chat</button>
                </div>
              )}
            </div>
          </div>

          {/* History */}
          {historyOpen && (
            <div className="chat-history">
              {messages.map((msg, idx) => (
                <p key={idx}><strong>{msg.sender === 'bot' ? 'AI:' : 'You:'}</strong> {msg.text}</p>
              ))}
            </div>
          )}

          {/* Messages */}
          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="welcome-message">
                <p><strong>Hello! 👋</strong> I’m your AI assistant for the <strong>Physical AI and Humanoid Robotics</strong> book.</p>
                <ul>
                  <li>Ask me questions about any topic</li>
                  <li>Select text and click <strong>“Select Text”</strong> to ask about it</li>
                  <li>Get answers backed by the book’s content</li>
                </ul>
              </div>
            )}

            {messages.map((message, index) => (
              <div key={index} className={`${message.sender}-message-wrapper`}>
                <div className={`message ${message.sender}-message`}>
                  <p>{message.text}</p>
                  <span className="message-time">
                    {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </span>
                </div>
                {message.sender === 'bot' && (
                  <div className="copy-icon-wrapper">
                    {copiedIndex === index
                      ? <span className="copy-icon">✔</span>
                      : <FiCopy className="copy-icon" onClick={() => handleCopy(message.text, index)} title="Copy Answer" />}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className="message bot-message">
                <p>AI is typing...</p>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Type your message..."
              rows="2"
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !inputValue.trim()} className="send-button">
              <FiSend />
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
