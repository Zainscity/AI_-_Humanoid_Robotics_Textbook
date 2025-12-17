import React from 'react';
import { ChatProvider } from '../context/ChatContext';
import { AuthProvider } from '../context/AuthContext';
import ChatWidget from '../components/ChatWidget';

function Root({children}) {
  return (
    <AuthProvider>
      <ChatProvider>
        {children}
        <ChatWidget />
      </ChatProvider>
    </AuthProvider>
  );
}

export default Root;
