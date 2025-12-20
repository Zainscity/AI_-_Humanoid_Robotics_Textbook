import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { motion, AnimatePresence } from 'framer-motion';
import { useColorMode } from '@docusaurus/theme-common';
import clsx from 'clsx';
import API_BASE_URL from '../../config';
import * as api from '../../frontend/src/services/api';
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

const ChatView = ({ activeConversation, onConversationCreated, onSidebarToggle, isMobile }) => {
    const { colorMode } = useColorMode();
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [isApiLoading, setIsApiLoading] = useState(false); // Use a different name to avoid conflict
    const [isFetchingHistory, setIsFetchingHistory] = useState(false);
    const messagesEndRef = useRef(null);

    // Scroll to bottom of messages
    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        const fetchMessages = async () => {
            if (activeConversation) {
                setIsFetchingHistory(true);
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
                        setIsFetchingHistory(false);
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

        const token = localStorage.getItem('token');
        if (!token) return;

        const queryText = input;
        const userMessage = { content: queryText, is_from_user: true, id: Date.now() };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setIsApiLoading(true);

        try {
            const response = await api.query(queryText, activeConversation, token);
            const botMessage = { content: response.message, is_from_user: false, id: response.message_id || Date.now() + 1 };
            
            // Add bot message to the existing list of messages
            setMessages(prev => [...prev, botMessage]);

            // If it was a new conversation, update the parent's state
            if (!activeConversation && response.conversation_id) {
                onConversationCreated(response.conversation_id);
            }
        } catch (error) {
            console.error("Failed to send message:", error);
            const errorMessage = { content: "Sorry, I couldn't get a response. Please try again.", is_from_user: false, id: Date.now() + 1 };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsApiLoading(false);
        }
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
                {isFetchingHistory ? (
                    <div className={styles.welcomeMessage}>Loading history...</div>
                ) : messages.length > 0 ? (
                    <AnimatePresence>
                        {messages.map((msg) => (
                            <motion.div
                                key={msg.id}
                                className={clsx(styles.messageBubble, msg.is_from_user ? styles.user : styles.bot, styles[colorMode])}
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
                {isApiLoading && (
                     <motion.div
                        className={clsx(styles.messageBubble, styles.bot, styles[colorMode])}
                        variants={bubbleVariants}
                        initial="hidden"
                        animate="visible"
                    >
                       <div className="typing-indicator">
                            <span /><span /><span />
                        </div>
                    </motion.div>
                )}
                <div ref={messagesEndRef} />
            </div>

            <div className={clsx(styles.inputArea, styles[colorMode])}>
                <form onSubmit={handleSendMessage} className={styles.inputForm}>
                    <textarea
                        value={input}
                        onChange={(e) => setInput(e.target.value)}
                        placeholder="Ask anything about the book..."
                        className={clsx(styles.messageInput, styles[colorMode])}
                        rows="1"
                        onKeyDown={(e) => {
                            if (e.key === 'Enter' && !e.shiftKey) {
                                e.preventDefault();
                                handleSendMessage(e);
                            }
                        }}
                    />
                    <button type="submit" className={styles.sendButton} disabled={!input.trim() || isApiLoading} aria-label="Send message">
                        <SendIcon />
                    </button>
                </form>
            </div>
        </motion.section>
    );
};

export default ChatView;
