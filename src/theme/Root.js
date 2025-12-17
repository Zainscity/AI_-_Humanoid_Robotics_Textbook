import React from 'react';
import { ChatProvider } from '../../frontend/src/context/ChatContext';
import { AuthProvider } from '../../frontend/src/context/AuthContext';
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
