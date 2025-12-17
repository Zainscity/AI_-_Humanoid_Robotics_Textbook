import React, { useState } from 'react';
import Layout from '@theme/Layout';
import axios from 'axios';
import clsx from 'clsx';
import './../css/login.css'; // Import the new CSS file

const LoginPage = () => {
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [message, setMessage] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      if (isLogin) {
        const response = await axios.post('http://localhost:8000/auth/token', new URLSearchParams({
          username: email,
          password: password,
        }));
        localStorage.setItem('token', response.data.access_token);
        setMessage('Login successful!');
        window.location.href = '/';
      } else {
        await axios.post('http://localhost:8000/auth/register', { email, password });
        setMessage('Registration successful! Please log in.');
      }
    } catch (error) {
      setMessage(error.response.data.detail);
    }
  };

  return (
    <Layout title="Login">
      <div className="login-page">
        <div>
          <div className="login-tabs">
            <button className={clsx({ 'active': isLogin })} onClick={() => setIsLogin(true)}>Login</button>
            <button className={clsx({ 'active': !isLogin })} onClick={() => setIsLogin(false)}>Register</button>
          </div>
          <form onSubmit={handleSubmit} className="login-form">
            <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} placeholder="Email" required />
            <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} placeholder="Password" required />
            <button type="submit">{isLogin ? 'Login' : 'Register'}</button>
          </form>
          {message && <p style={{ marginTop: '1rem', textAlign: 'center' }}>{message}</p>}
        </div>
      </div>
    </Layout>
  );
};

export default LoginPage;
