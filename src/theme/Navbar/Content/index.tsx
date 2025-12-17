import React, {type ReactNode, useState, useEffect} from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import {useLocation} from '@docusaurus/router';

type Props = WrapperProps<typeof ContentType>;

const useIsLoggedIn = () => {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const token = localStorage.getItem('token');
    setIsLoggedIn(!!token);
  }, [location.pathname]);

  return isLoggedIn;
};

export default function ContentWrapper(props: Props): ReactNode {
  const isLoggedIn = useIsLoggedIn();
  const {rightItems} = props;
  const filteredRightItems = rightItems ? rightItems.filter(item => {
    if (item.label === 'Login') {
      return !isLoggedIn;
    }
    if (item.label === 'Profile' || item.label === 'Logout') {
      return isLoggedIn;
    }
    return true;
  }) : [];

  const handleLogout = () => {
    localStorage.removeItem('token');
    window.location.href = '/';
  };

  return (
    <>
      <Content {...props} rightItems={filteredRightItems.map(item => {
        if (item.label === 'Logout') {
          return {
            ...item,
            onClick: handleLogout,
            to: '#' // Prevent navigation
          };
        }
        return item;
      })} />
    </>
  );
}
