import React, { createContext, useState } from 'react';

export const ChatContext = createContext();

export const ChatProvider = ({ children }) => {
  const [messages, setMessages] = useState([]);
  const [isWidgetOpen, setIsWidgetOpen] = useState(false);

  return (
    <ChatContext.Provider value={{ messages, setMessages, isWidgetOpen, setIsWidgetOpen }}>
      {children}
    </ChatContext.Provider>
  );
};
