import React from 'react';
import { motion } from 'framer-motion';
import { useColorMode } from '@docusaurus/theme-common';
import clsx from 'clsx';
import styles from './styles.module.css';

// --- Helper to generate a simple color from a string ---
const stringToColor = (str) => {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    hash = str.charCodeAt(i) + ((hash << 5) - hash);
  }
  let color = '#';
  for (let i = 0; i < 3; i++) {
    const value = (hash >> (i * 8)) & 0xFF;
    color += ('00' + value.toString(16)).substr(-2);
  }
  return color;
};

// --- Avatar Placeholder ---
const Avatar = ({ email }) => {
  const initial = email ? email.charAt(0).toUpperCase() : '?';
  const bgColor = "stringToColor"(email || 'default');

  return (
    <div className={styles.avatar} style={{ backgroundColor: bgColor }}>
      {initial}
    </div>
  );
};

const PlusIcon = () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round" strokeLinejoin="round">
        <line x1="12" y1="5" x2="12" y2="19"></line>
        <line x1="5" y1="12" x2="19" y2="12"></line>
    </svg>
);


const ProfileSidebar = ({
  user,
  conversations,
  activeConversation,
  onConversationSelect,
  onNewConversation,
  onLogout,
  className, // For responsive wrapper
}) => {
  const { colorMode } = useColorMode();

  const containerVariants = {
    hidden: { x: '-100%', opacity: 0 },
    visible: { x: 0, opacity: 1, transition: { type: 'spring', stiffness: 300, damping: 30 } },
    exit: { x: '-100%', opacity: 0, transition: { duration: 0.3 } }
  };

  return (
    <motion.aside 
        className={clsx(styles.sidebar, styles[colorMode], className)}
        variants={containerVariants}
        initial="hidden"
        animate="visible"
        exit="exit"
    >
      {/* --- Profile Card --- */}
      <div className={clsx(styles.profileCard, styles[colorMode])}>
        <Avatar email={user.email} />
        <p className={styles.userEmail}>{user.email}</p>
        <button onClick={onLogout} className={clsx(styles.logoutButton, styles[colorMode])}>
          Logout
        </button>
      </div>

      {/* --- Conversation History --- */}
      <div className={styles.historySection}>
        <div className={styles.historyHeader}>
            <h2 className={styles.historyTitle}>History</h2>
            <button onClick={onNewConversation} className={styles.newConvoButton} aria-label="New Conversation">
                <PlusIcon />
            </button>
        </div>
        <ul className={styles.conversationList}>
          {conversations.map((convo) => (
            <motion.li
              key={convo.id}
              className={clsx(styles.conversationItem, { [styles.active]: activeConversation === convo.id })}
              onClick={() => onConversationSelect(convo.id)}
              whileHover={{ scale: 1.02 }}
              whileTap={{ scale: 0.98 }}
            >
              <div className={styles.convoDate}>
                {new Date(convo.created_at).toLocaleString()}
              </div>
            </motion.li>
          ))}
        </ul>
      </div>
    </motion.aside>
  );
};

export default ProfileSidebar;
