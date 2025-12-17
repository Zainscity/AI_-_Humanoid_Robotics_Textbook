import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import axios from 'axios';
import { useHistory } from '@docusaurus/router';
import './../css/profile.css'; // Import the new CSS file

const ProfilePage = () => {
  const [user, setUser] = useState(null);
  const [conversations, setConversations] = useState([]);
  const [messages, setMessages] = useState([]);
  const [activeConversation, setActiveConversation] = useState(null);
  const history = useHistory();

  useEffect(() => {
    const token = localStorage.getItem('token');
    if (!token) {
      history.push('/login');
    } else {
      // Fetch user data and conversations
      const fetchUserData = async () => {
        try {
          const userResponse = await axios.get('http://localhost:8000/auth/me', {
            headers: { Authorization: `Bearer ${token}` },
          });
          setUser(userResponse.data);

          const conversationsResponse = await axios.get('http://localhost:8000/history/conversations', {
            headers: { Authorization: `Bearer ${token}` },
          });
          setConversations(conversationsResponse.data);
        } catch (error) {
          console.error(error);
          localStorage.removeItem('token');
          history.push('/login');
        }
      };
      fetchUserData();
    }
  }, [history]);

  const handleLogout = () => {
    localStorage.removeItem('token');
    history.push('/');
  };
  
  const handleNewConversation = () => {
      setActiveConversation(null);
      setMessages([]);
  }

  const fetchMessages = async (conversationId) => {
    const token = localStorage.getItem('token');
    if (token) {
      const msgs = await axios.get(`http://localhost:8000/history/conversations/${conversationId}/messages`, {
        headers: { Authorization: `Bearer ${token}` },
      });
      setMessages(msgs.data);
      setActiveConversation(conversationId);
    }
  };

  if (!user) {
    return <Layout title="Profile"><div>Loading...</div></Layout>;
  }

  return (
    <Layout title="Profile">
      <div className="profile-page">
        <div className="profile-sidebar">
            <h1>Profile</h1>
            <p>Email: {user.email}</p>
            <button onClick={handleLogout} className="profile-button">Logout</button>
            <h2 style={{ marginTop: '2rem' }}>Conversation History</h2>
            <button onClick={handleNewConversation} className="profile-button" style={{ marginBottom: '1rem' }}>New Conversation</button>
            <ul className="conversation-list">
              {conversations.map((convo) => (
                <li key={convo.id} onClick={() => fetchMessages(convo.id)} className={clsx({ 'active': activeConversation === convo.id })}>
                  <p>Conversation ID: {convo.id}</p>
                  <p>Created at: {new Date(convo.created_at).toLocaleString()}</p>
                </li>
              ))}
            </ul>
        </div>
        <div className="profile-content">
            <h2>Messages</h2>
            <div className="profile-messages">
                {messages.map((msg, index) => (
                  <div key={index} className={clsx('profile-message', msg.is_from_user ? 'user' : 'bot')}>
                    <p>{msg.content}</p>
                  </div>
                ))}
            </div>
        </div>
      </div>
    </Layout>
  );
};

export default ProfilePage;
