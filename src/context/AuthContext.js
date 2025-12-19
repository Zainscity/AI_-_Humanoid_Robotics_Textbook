import React, { createContext, useState, useEffect } from 'react';
import useIsBrowser from '@docusaurus/useIsBrowser';

export const AuthContext = createContext({
  isLoggedIn: false,
  login: (token) => {},
  logout: () => {},
});

export const AuthProvider = ({ children }) => {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const isBrowser = useIsBrowser();

  useEffect(() => {
    if (isBrowser) {
      const token = localStorage.getItem('token');
      if (token) {
        setIsLoggedIn(true);
      } else {
        setIsLoggedIn(false);
      }
    }
  }, [isBrowser]);

  const login = (token) => {
    localStorage.setItem('token', token);
    setIsLoggedIn(true);
  };

  const logout = () => {
    localStorage.removeItem('token');
    setIsLoggedIn(false);
  };

  return (
    <AuthContext.Provider value={{ isLoggedIn, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
};