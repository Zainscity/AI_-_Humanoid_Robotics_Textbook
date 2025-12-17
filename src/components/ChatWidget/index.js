import React, { useState, useContext, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { ChatContext } from '../../../frontend/src/context/ChatContext';
import * as api from '../../../frontend/src/services/api';
import { useLocation } from '@docusaurus/router';
import styles from './styles.css';

const useIsLoggedIn = () => {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const token = localStorage.getItem('token');
    setIsLoggedIn(!!token);
  }, [location.pathname]);

  return isLoggedIn;
};

const ChatWidget = () => {
  const { messages, setMessages, isWidgetOpen, setIsWidgetOpen } = useContext(ChatContext);
  const [input, setInput] = useState('');
  const [conversations, setConversations] = useState([]);
  const [activeConversation, setActiveConversation] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const isLoggedIn = useIsLoggedIn();
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async () => {
    if (input.trim() && isLoggedIn) {
      const token = localStorage.getItem('token');
      const newMessages = [...messages, { text: input, from: 'user' }];
      setMessages(newMessages);
      const queryText = input;
      setInput('');
      setIsLoading(true);
      const response = await api.query(queryText, activeConversation, token);
      setIsLoading(false);
      setMessages([...newMessages, { text: response.message, from: 'bot' }]);
      if (!activeConversation) {
        setActiveConversation(response.conversation_id);
      }
    }
  };
  
  if (!isLoggedIn) {
    return null;
  }

  return (
    <div className="chat-widget-container">
      <AnimatePresence>
        {isWidgetOpen && (
          <motion.div
            className="chat-window"
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: 20 }}
          >
            <div className="chat-header">
              <div className="chat-header-title">
                <div className="ai-icon"></div>
                <span>AI Agent</span>
              </div>
              <button className="close-button" onClick={() => setIsWidgetOpen(false)}>
                &times;
              </button>
            </div>
            <div className="chat-body">
              {messages.map((msg, index) => (
                <div key={index} className={`message-bubble ${msg.from}`}>
                  <div className="message-content">{msg.text}</div>
                </div>
              ))}
              {isLoading && (
                <div className="message-bubble bot">
                  <div className="typing-indicator">
                    <span />
                    <span />
                    <span />
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className="chat-input-area">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSend()}
                placeholder="Write your message..."
              />
              <button className="send-button" onClick={handleSend}>
                &#10148;
              </button>
            </div>
          </motion.div>
        )}
      </AnimatePresence>
      <button className="chat-bubble" onClick={() => setIsWidgetOpen(!isWidgetOpen)}>
        <span className="notification-badge">1</span>
        &#128172;
      </button>
    </div>
  );
};

export default ChatWidget;
