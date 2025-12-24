from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import os
import json
from typing import Dict, Any

app = FastAPI()

# Add CORS middleware to allow iframe embedding
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for iframe embedding (including from Docusaurus)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    allow_origin_regex=".*"  # Allow regex patterns as well
)

# Serve static files (frontend assets) if they exist
if os.path.exists("static"):
    app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    """Serve the main chatbot UI page - this is the route that was causing the issue"""
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Physical AI & Humanoid Robotics Assistant</title>
        <style>
            * {
                margin: 0;
                padding: 0;
                box-sizing: border-box;
            }

            body {
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
                background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
                height: 100vh;
                overflow: hidden;
                display: flex;
                justify-content: center;
                align-items: center;
            }

            #chat-container {
                display: flex;
                flex-direction: column;
                height: 90vh;
                width: 90vw;
                max-width: 900px;
                background: white;
                border-radius: 16px;
                overflow: hidden;
                box-shadow: 0 10px 30px rgba(0,0,0,0.15);
                border: 1px solid #e1e5e9;
            }

            #chat-header {
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                padding: 1.2rem;
                text-align: center;
                font-weight: 600;
                font-size: 1.3rem;
                box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            }

            #chat-messages {
                flex: 1;
                overflow-y: auto;
                padding: 1.5rem;
                display: flex;
                flex-direction: column;
                gap: 1rem;
                background: #fafafa;
            }

            .message {
                max-width: 80%;
                padding: 1rem;
                border-radius: 18px;
                margin-bottom: 0.75rem;
                position: relative;
                animation: fadeIn 0.3s ease-out;
                line-height: 1.5;
            }

            @keyframes fadeIn {
                from { opacity: 0; transform: translateY(10px); }
                to { opacity: 1; transform: translateY(0); }
            }

            .user-message {
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                align-self: flex-end;
                border-bottom-right-radius: 4px;
            }

            .bot-message {
                background: white;
                color: #333;
                align-self: flex-start;
                border: 1px solid #e9ecef;
                border-bottom-left-radius: 4px;
            }

            #input-container {
                display: flex;
                padding: 1.2rem;
                background: white;
                border-top: 1px solid #e9ecef;
                box-shadow: 0 -2px 10px rgba(0,0,0,0.05);
            }

            #message-input {
                flex: 1;
                padding: 0.9rem 1.2rem;
                border: 2px solid #e9ecef;
                border-radius: 24px;
                font-size: 1rem;
                outline: none;
                transition: border-color 0.3s ease;
            }

            #message-input:focus {
                border-color: #667eea;
            }

            #send-button {
                margin-left: 0.8rem;
                padding: 0.9rem 1.5rem;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                border: none;
                border-radius: 24px;
                cursor: pointer;
                font-size: 1rem;
                font-weight: 500;
                transition: transform 0.2s ease, box-shadow 0.2s ease;
            }

            #send-button:hover {
                transform: translateY(-2px);
                box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
            }

            .typing-indicator {
                display: none;
                padding: 1rem;
                color: #6c757d;
                font-style: italic;
                font-size: 0.9rem;
                align-self: flex-start;
                background: white;
                border-radius: 18px;
                border: 1px solid #e9ecef;
            }

            .message-time {
                font-size: 0.7rem;
                opacity: 0.7;
                margin-top: 0.3rem;
                text-align: right;
            }

            @media (max-width: 768px) {
                body {
                    margin: 0;
                    height: 100vh;
                }

                #chat-container {
                    height: 100vh;
                    width: 100vw;
                    border-radius: 0;
                    max-width: none;
                }

                #chat-header {
                    padding: 1rem;
                    font-size: 1.1rem;
                }

                #chat-messages {
                    padding: 1rem;
                }

                .message {
                    max-width: 85%;
                    padding: 0.8rem;
                }

                #input-container {
                    padding: 1rem;
                }

                #message-input {
                    padding: 0.8rem;
                }

                #send-button {
                    padding: 0.8rem 1.2rem;
                }
            }
        </style>
    </head>
    <body>
        <div id="chat-container">
            <div id="chat-header">
                🤖 Physical AI & Humanoid Robotics Assistant
            </div>
            <div id="chat-messages">
                <div class="message bot-message">
                    <div>Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics book. I can help answer questions about:</div>
                    <div style="margin-top: 0.5rem; font-size: 0.9em; opacity: 0.8;">
                        • Embodied AI systems<br>
                        • Humanoid robotics<br>
                        • Sensorimotor learning<br>
                        • Physical AI concepts
                    </div>
                    <div class="message-time">Just now</div>
                </div>
                <div class="typing-indicator" id="typing">AI is typing...</div>
            </div>
            <div id="input-container">
                <input type="text" id="message-input" placeholder="Ask about Physical AI, Humanoid Robotics..." />
                <button id="send-button">Send</button>
            </div>
        </div>

        <script>
            const messageInput = document.getElementById('message-input');
            const sendButton = document.getElementById('send-button');
            const chatMessages = document.getElementById('chat-messages');
            const typingIndicator = document.getElementById('typing');

            function getCurrentTime() {
                const now = new Date();
                return now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
            }

            function addMessage(text, isUser) {
                const messageDiv = document.createElement('div');
                messageDiv.classList.add('message');
                messageDiv.classList.add(isUser ? 'user-message' : 'bot-message');

                const textDiv = document.createElement('div');
                textDiv.textContent = text;

                const timeDiv = document.createElement('div');
                timeDiv.classList.add('message-time');
                timeDiv.textContent = getCurrentTime();

                messageDiv.appendChild(textDiv);
                messageDiv.appendChild(timeDiv);

                chatMessages.insertBefore(messageDiv, typingIndicator);
                chatMessages.scrollTop = chatMessages.scrollHeight;
            }

            async function sendMessage() {
                const message = messageInput.value.trim();
                if (!message) return;

                // Add user message to chat
                addMessage(message, true);
                messageInput.value = '';

                // Show typing indicator
                typingIndicator.style.display = 'block';
                chatMessages.scrollTop = chatMessages.scrollHeight;

                try {
                    // Simulate AI response (in a real app, this would call your AI backend)
                    // In a real implementation, you would make a fetch request to an API endpoint
                    setTimeout(() => {
                        typingIndicator.style.display = 'none';
                        const responses = [
                            "That's a great question about the book! The Physical AI and Humanoid Robotics book covers advanced concepts in embodied AI.",
                            "I can help you understand that topic better. Check chapter 3 for more details on sensorimotor learning.",
                            "Interesting question! The book discusses this in the context of physical intelligence and embodied cognition.",
                            "Great point! The book explores how physical AI systems can learn from interaction with their environment through active sensing.",
                            "I recommend checking the section on humanoid robotics for more information on motor control and learning.",
                            "This concept is central to the book. Physical AI combines machine learning with real-world physical interaction.",
                            "The book explains this in the context of reinforcement learning for robotics applications and embodied agents.",
                            "That's covered in the chapters about morphological computation and how body shape influences intelligence."
                        ];
                        const randomResponse = responses[Math.floor(Math.random() * responses.length)];
                        addMessage(randomResponse, false);
                    }, 1000 + Math.random() * 1000); // Random delay to simulate thinking
                } catch (error) {
                    typingIndicator.style.display = 'none';
                    addMessage("Sorry, I encountered an error processing your request. Please try again.", false);
                }
            }

            sendButton.addEventListener('click', sendMessage);
            messageInput.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    sendMessage();
                }
            });

            // Focus input field on load
            messageInput.focus();
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

@app.post("/chat")
async def chat_endpoint(request: Request):
    """API endpoint for chat functionality - This should connect to the real RAG backend"""
    try:
        body = await request.json()
        user_message = body.get("message", "")

        # In a real implementation, you would process the message with your AI model
        # For now, return an error indicating this is not implemented
        return JSONResponse(content={
            "response": "Chat endpoint not implemented in this service. Use /ask endpoint for RAG queries.",
            "status": "error"
        }, status_code=501)
    except Exception as e:
        return JSONResponse(content={
            "response": "Sorry, I encountered an error processing your request.",
            "status": "error",
            "error": str(e)
        }, status_code=500)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)