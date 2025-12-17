import React from 'react';
import { ChatProvider } from '../../frontend/src/context/ChatContext';
import ChatWidget from '../components/ChatWidget';

function Root({children}) {
  return (
    <ChatProvider>
      {children}
      <ChatWidget />
    </ChatProvider>
  );
}

export default Root;
