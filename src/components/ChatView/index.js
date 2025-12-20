import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { motion, AnimatePresence } from 'framer-motion';
import { useColorMode } from '@docusaurus/theme-common';
import clsx from 'clsx';
import API_BASE_URL from '../../config';
import styles from './styles.module.css';

const SendIcon = () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <line x1="22" y1="2" x2="11" y2="13"></line>
        <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
    </svg>
);

const MenuIcon = () => (
    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <line x1="3" y1="12" x2="21" y2="12"></line>
        <line x1="3" y1="6" x2="21" y2="6"></line>
        <line x1="3" y1="18" x2="21" y2="18"></line>
    </svg>
);

const ChatView = ({ activeConversation, onSidebarToggle, isMobile }) => {
    const { colorMode } = useColorMode();
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const messagesEndRef = useRef(null);

    // Scroll to bottom of messages
    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        const fetchMessages = async () => {
            if (activeConversation) {
                setIsLoading(true);
                const token = localStorage.getItem('token');
                if (token) {
                    try {
                        const res = await axios.get(`${API_BASE_URL}/history/conversations/${activeConversation}/messages`, {
                            headers: { Authorization: `Bearer ${token}` },
                        });
                        setMessages(res.data);
                    } catch (error) {
                        console.error("Failed to fetch messages:", error);
                    } finally {
                        setIsLoading(false);
                    }
                }
            } else {
                setMessages([]);
            }
        };
        fetchMessages();
    }, [activeConversation]);

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    const handleSendMessage = async (e) => {
        e.preventDefault();
        if (!input.trim()) return;

        // Note: The backend logic for sending a message and getting a response
        // is not fully defined in the original code. This is a placeholder implementation.
        // It optimistically adds the user's message and then simulates a bot response.
        const userMessage = { content: input, is_from_user: true, id: Date.now() };
        setMessages(prev => [...prev, userMessage]);
        setInput('');

        // Simulate bot response
        setTimeout(() => {
            const botMessage = { content: "This is a simulated response.", is_from_user: false, id: Date.now() + 1 };
            setMessages(prev => [...prev, botMessage]);
        }, 1000);
    };

    const bubbleVariants = {
        hidden: { opacity: 0, y: 20 },
        visible: { opacity: 1, y: 0, transition: { duration: 0.3 } },
    };

    return (
        <motion.section 
            className={clsx(styles.chatContainer, styles[colorMode])}
            initial={{ opacity: 0, y: 50 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: 0.2 }}
        >
            {isMobile && <button onClick={onSidebarToggle} className={styles.menuButton} aria-label="Toggle sidebar"><MenuIcon /></button>}
            
            <div className={styles.messagesArea}>
                {isLoading ? (
                    <div>Loading messages...</div>
                ) : messages.length > 0 ? (
                    <AnimatePresence>
                        {messages.map((msg) => (
                            <motion.div
                                key={msg.id}
                                className={clsx(styles.messageBubble, styles[msg.is_from_user ? 'user' : 'bot'], styles[colorMode])}
                                variants={bubbleVariants}
                                initial="hidden"
                                animate="visible"
                                layout
                            >
                                {msg.content}
                            </motion.div>
                        ))}
                    </AnimatePresence>
                ) : (
                    <div className={styles.welcomeMessage}>
                        <h2>Welcome!</h2>
                        <p>Select a conversation or start a new one.</p>
                    </div>
                )}
                <div ref={messagesEndRef} />
            </div>

            <div className={clsx(styles.inputArea, styles[colorMode])}>
                <form onSubmit={handleSendMessage} className={styles.inputForm}>
                    <textarea
                        value={input}
                        onChange={(e) => setInput(e.target.value)}
                        placeholder={activeConversation ? "Type your message..." : "Please select a conversation first"}
                        className={clsx(styles.messageInput, styles[colorMode])}
                        rows="1"
                        disabled={!activeConversation}
                        onKeyDown={(e) => {
                            if (e.key === 'Enter' && !e.shiftKey) {
                                e.preventDefault();
                                handleSendMessage(e);
                            }
                        }}
                    />
                    <button type="submit" className={styles.sendButton} disabled={!input.trim() || !activeConversation} aria-label="Send message">
                        <SendIcon />
                    </button>
                </form>
            </div>
        </motion.section>
    );
};

export default ChatView;
