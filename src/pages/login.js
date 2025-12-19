import React, { useState, useContext, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useColorMode } from '@docusaurus/theme-common';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { motion, AnimatePresence } from 'framer-motion';
import axios from 'axios';
import clsx from 'clsx';
import { AuthContext } from '../context/AuthContext';
import API_BASE_URL from '../config';
import styles from './login.module.css';

// --- SVG Icons for inputs ---
const MailIcon = () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"></path>
      <polyline points="22,6 12,13 2,6"></polyline>
    </svg>
  );
  
  const LockIcon = () => (
    <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
      <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
    </svg>
  );
  
  const EyeIcon = () => (
      <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z" />
        <circle cx="12" cy="12" r="3" />
      </svg>
  );
  
  const EyeOffIcon = () => (
      <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <path d="M9.88 9.88a3 3 0 1 0 4.24 4.24" />
          <path d="M10.73 10.73C12.55 9.42 15.8 8.01 19 8a9 9 0 0 1 3 12" />
          <path d="M2 12s3-7 10-7a9 9 0 0 1 2.24.36" />
          <path d="m2 2 20 20" />
      </svg>
  );

// --- Inner Component (Client-Only) ---
const LoginPageContent = () => {
    const { colorMode } = useColorMode();
    const [isLogin, setIsLogin] = useState(true);
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [showPassword, setShowPassword] = useState(false);
    const [message, setMessage] = useState('');
    const [isSuccess, setIsSuccess] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const { login } = useContext(AuthContext);
  
    const handleSubmit = async (e) => {
      e.preventDefault();
      setIsLoading(true);
      setMessage('');
      try {
        if (isLogin) {
          const response = await axios.post(`${API_BASE_URL}/auth/token`, new URLSearchParams({
            username: email,
            password: password,
          }));
          login(response.data.access_token);
          setIsSuccess(true);
          setMessage('Login successful! Redirecting...');
          setTimeout(() => window.location.href = '/', 1000);
        } else {
          await axios.post(`${API_BASE_URL}/auth/register`, { email, password });
          setIsSuccess(true);
          setMessage('Registration successful! Please log in.');
          setIsLogin(true); // Switch to login tab after registration
        }
      } catch (error) {
        setIsSuccess(false);
        setMessage(error.response?.data?.detail || 'An unexpected error occurred.');
      } finally {
          setIsLoading(false);
      }
    };
  
    const cardVariants = {
      hidden: { opacity: 0, y: 50, filter: 'blur(10px)' },
      visible: { opacity: 1, y: 0, filter: 'blur(0px)', transition: { duration: 0.5, ease: 'easeOut' } },
    };
  
    const formVariants = {
      hidden: { opacity: 0, x: -20 },
      visible: { opacity: 1, x: 0, transition: { duration: 0.4, ease: 'easeOut' } },
      exit: { opacity: 0, x: 20, transition: { duration: 0.2, ease: 'easeIn' } },
    }

    return (
        <motion.div
            className={clsx(styles.loginPage, styles[colorMode])}
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ duration: 0.5 }}
        >
            <motion.div
                className={clsx(styles.authCard, styles[colorMode])}
                variants={cardVariants}
                initial="hidden"
                animate="visible"
            >
                {/* --- Tab Switcher --- */}
                <div className={styles.tabContainer}>
                    <button
                        onClick={() => setIsLogin(true)}
                        className={clsx(styles.tabButton, { [styles.active]: isLogin })}
                        aria-pressed={isLogin}
                    >
                        {isLogin && (
                            <motion.div
                                className={clsx(styles.activeTabPill, styles[colorMode])}
                                layoutId="active-pill"
                                transition={{ type: 'spring', stiffness: 500, damping: 30 }}
                            />
                        )}
                        Login
                    </button>
                    <button
                        onClick={() => setIsLogin(false)}
                        className={clsx(styles.tabButton, { [styles.active]: !isLogin })}
                        aria-pressed={!isLogin}
                    >
                        {!isLogin && (
                            <motion.div
                                className={clsx(styles.activeTabPill, styles[colorMode])}
                                layoutId="active-pill"
                                transition={{ type: 'spring', stiffness: 500, damping: 30 }}
                            />
                        )}
                        Register
                    </button>
                </div>

                {/* --- Form Area --- */}
                <AnimatePresence mode="wait">
                    <motion.form
                        key={isLogin ? 'login' : 'register'}
                        className={styles.authForm}
                        onSubmit={handleSubmit}
                        variants={formVariants}
                        initial="hidden"
                        animate="visible"
                        exit="exit"
                    >
                        <div className={styles.inputGroup}>
                            <span className={styles.inputIcon}><MailIcon /></span>
                            <input
                                type="email"
                                value={email}
                                onChange={(e) => setEmail(e.target.value)}
                                placeholder="Email"
                                required
                                className={clsx(styles.formInput, styles[colorMode])}
                                aria-label="Email Address"
                            />
                        </div>

                        <div className={styles.inputGroup}>
                            <span className={styles.inputIcon}><LockIcon /></span>
                            <input
                                type={showPassword ? 'text' : 'password'}
                                value={password}
                                onChange={(e) => setPassword(e.target.value)}
                                placeholder="Password"
                                required
                                className={clsx(styles.formInput, styles[colorMode])}
                                aria-label="Password"
                            />
                            <button
                                type="button"
                                onClick={() => setShowPassword(!showPassword)}
                                className={styles.passwordToggle}
                                aria-label={showPassword ? "Hide password" : "Show password"}
                            >
                                {showPassword ? <EyeOffIcon /> : <EyeIcon />}
                            </button>
                        </div>

                        <button type="submit" className={styles.submitButton} disabled={isLoading}>
                            {isLoading ? 'Processing...' : (isLogin ? 'Login' : 'Create Account')}
                        </button>
                    </motion.form>
                </AnimatePresence>

                {/* --- Message Display --- */}
                {message && (
                    <div className={clsx(styles.message, isSuccess ? styles.success : styles.error)}>
                        {message}
                    </div>
                )}
            </motion.div>
        </motion.div>
    );
};

// --- Main Page Component (Wrapper) ---
const LoginPage = () => {
    return (
        <Layout title="Login / Register" noFooter>
            <BrowserOnly fallback={<div>Loading...</div>}>
                {() => <LoginPageContent />}
            </BrowserOnly>
        </Layout>
    );
};

export default LoginPage;
