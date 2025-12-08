import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbox.module.css'; // Assuming you'll create this CSS module

function Chatbox() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage = { id: Date.now(), text: input, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');

    try {
      // Replace with your actual RAG backend URL
      const response = await fetch('http://localhost:8000/api/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: input }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage = { id: Date.now() + 1, text: data.answer, sender: 'bot', source: data.source };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error("Error fetching RAG response:", error);
      const errorMessage = { id: Date.now() + 1, text: "Error: Could not get a response.", sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    }
  };

  return (
    <div className={styles.chatboxContainer}>
      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            Ask me anything about ROS 2 Graph Basics, Python Agents, or URDF!
          </div>
        )}
        {messages.map((msg) => (
          <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
            <p>{msg.text}</p>
            {msg.sender === 'bot' && msg.source && (
              <span className={styles.source}>({msg.source})</span>
            )}
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Type your question..."
          className={styles.textInput}
        />
        <button type="submit" className={styles.sendButton}>Send</button>
      </form>
    </div>
  );
}

export default Chatbox;
