import React, { useState } from 'react';
import Layout from '@theme/Layout';
import axios from 'axios';

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
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '80vh' }}>
        <div>
          <div style={{ display: 'flex', justifyContent: 'center', marginBottom: '1rem' }}>
            <button onClick={() => setIsLogin(true)} style={{ padding: '0.5rem 1rem', border: 'none', background: isLogin ? '#333' : '#eee', color: isLogin ? 'white' : 'black' }}>Login</button>
            <button onClick={() => setIsLogin(false)} style={{ padding: '0.5rem 1rem', border: 'none', background: !isLogin ? '#333' : '#eee', color: !isLogin ? 'white' : 'black' }}>Register</button>
          </div>
          <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '1rem', width: '300px' }}>
            <input type="email" value={email} onChange={(e) => setEmail(e.target.value)} placeholder="Email" required style={{ padding: '0.5rem' }} />
            <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} placeholder="Password" required style={{ padding: '0.5rem' }} />
            <button type="submit" style={{ padding: '0.5rem', background: '#333', color: 'white', border: 'none' }}>{isLogin ? 'Login' : 'Register'}</button>
          </form>
          {message && <p style={{ marginTop: '1rem', textAlign: 'center' }}>{message}</p>}
        </div>
      </div>
    </Layout>
  );
};

export default LoginPage;
