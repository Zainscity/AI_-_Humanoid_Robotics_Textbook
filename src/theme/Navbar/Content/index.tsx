import React, {type ReactNode, useContext} from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { AuthContext } from '../../../frontend/src/context/AuthContext';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): ReactNode {
  const { isLoggedIn, logout } = useContext(AuthContext);
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
    logout();
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
