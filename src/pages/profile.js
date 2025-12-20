import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import axios from 'axios';
import { useHistory } from '@docusaurus/router';
import { useColorMode } from '@docusaurus/theme-common';
import { useWindowSize } from '@docusaurus/theme-common';
import BrowserOnly from '@docusaurus/BrowserOnly';
import clsx from 'clsx';
import API_BASE_URL from '../config';

import ProfileSidebar from '../components/ProfileSidebar';
import ChatView from '../components/ChatView';
import styles from '../css/profile-layout.module.css'; // New layout-specific styles

// Fallback for SSG
const ProfilePageContent = () => {
    const { colorMode } = useColorMode();
    const windowSize = useWindowSize();
    const history = useHistory();

    const [user, setUser] = useState(null);
    const [conversations, setConversations] = useState([]);
    const [activeConversation, setActiveConversation] = useState(null);
    
    // Responsive state
    const isMobile = windowSize.width < 997; // Docusaurus md breakpoint
    const [isSidebarOpen, setIsSidebarOpen] = useState(!isMobile);

    useEffect(() => {
        setIsSidebarOpen(!isMobile);
    }, [isMobile]);

    useEffect(() => {
        const token = localStorage.getItem('token');
        if (!token) {
            history.push('/login');
            return;
        }

        const fetchUserData = async () => {
            try {
                const userResponse = await axios.get(`${API_BASE_URL}/auth/me`, {
                    headers: { Authorization: `Bearer ${token}` },
                });
                setUser(userResponse.data);

                const conversationsResponse = await axios.get(`${API_BASE_URL}/history/conversations`, {
                    headers: { Authorization: `Bearer ${token}` },
                });
                setConversations(conversationsResponse.data);
            } catch (error) {
                console.error("Authentication or data fetching error:", error);
                localStorage.removeItem('token');
                history.push('/login');
            }
        };
        fetchUserData();
    }, [history]);

    const handleLogout = () => {
        localStorage.removeItem('token');
        history.push('/');
    };

    const handleNewConversation = () => {
        setActiveConversation(null);
        if (isMobile) setIsSidebarOpen(false);
    };

    const handleConversationSelect = (convoId) => {
        setActiveConversation(convoId);
        if (isMobile) setIsSidebarOpen(false);
    };

    if (!user) {
        // You can return a more sophisticated loading skeleton here
        return <div className={clsx(styles.pageContainer, styles[colorMode])}><div>Loading...</div></div>;
    }

    return (
        <div className={clsx(styles.pageContainer, styles[colorMode])}>
            <div className={styles.profileLayout}>
                {isSidebarOpen && (
                     <ProfileSidebar
                        user={user}
                        conversations={conversations}
                        activeConversation={activeConversation}
                        onConversationSelect={handleConversationSelect}
                        onNewConversation={handleNewConversation}
                        onLogout={handleLogout}
                        className={clsx(styles.sidebar, {[styles.mobileSidebar]: isMobile})}
                    />
                )}
                <main className={styles.mainContent}>
                    <ChatView 
                        activeConversation={activeConversation}
                        onSidebarToggle={() => setIsSidebarOpen(prev => !prev)}
                        isMobile={isMobile}
                    />
                </main>
            </div>
        </div>
    );
};


const ProfilePage = () => {
  return (
    <Layout title="Profile" noFooter>
      <BrowserOnly fallback={<div>Loading profile...</div>}>
        {() => <ProfilePageContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default ProfilePage;
