import { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import { v4 as uuidv4 } from 'uuid';
import './chat.css';

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);

  const chatBodyRef = useRef(null);

  // Auto-scroll to latest message
  useEffect(() => {
    chatBodyRef.current?.scrollTo(0, chatBodyRef.current.scrollHeight);
  }, [messages, loading]);

  const sendMessage = async () => {
    const trimmedInput = input.trim();
    if (!trimmedInput) return;

    const userMessage = { id: uuidv4(), sender: "user", text: trimmedInput };
    setMessages(prev => [...prev, userMessage]);
    setInput("");
    setLoading(true);

    try {
      console.log("Sending message:", trimmedInput);

      const res = await axios.post(
        "https://mahnoor-sabir-deploy-ai-book.hf.space/api/chat", // Adjust endpoint if needed
        { message: trimmedInput },
        { headers: { 'Content-Type': 'application/json' } }
      );

      console.log("Response received:", res);

      const botReply = res.data.reply || res.data.response || res.data.message || "No response from server";
      const botMessage = { id: uuidv4(), sender: "bot", text: botReply };

      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      console.error("Chat error:", err);

      const errorMsg = err.response?.data?.error || err.message || "Unable to reach server";
      setMessages(prev => [...prev, { 
        id: uuidv4(),
        sender: "bot",
        text: `Error: ${errorMsg}`
      }]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !loading) {
      sendMessage();
    }
  };

  return (
    <div className="chat-container">
      <button className="chat-button" onClick={() => setOpen(!open)}>
        💬 Chat
      </button>

      {open && (
        <div className="chat-box">
          <div className="chat-header">
            <strong>AI Assistant</strong>
          </div>

          <div className="chat-body" ref={chatBodyRef}>
            {messages.map((m) => (
              <div key={m.id} className={`bubble ${m.sender}`}>
                {m.text}
              </div>
            ))}
            {loading && (
              <div className="bubble bot">
                <em>Typing...</em>
              </div>
            )}
          </div>

          <div className="chat-input">
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Type message..."
              disabled={loading}
            />
            <button type="button" onClick={sendMessage} disabled={loading}>
              {loading ? "..." : "Send"}
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
