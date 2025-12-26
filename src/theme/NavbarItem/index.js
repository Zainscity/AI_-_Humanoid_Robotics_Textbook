import React, { useContext } from 'react';
import OriginalNavbarItem from '@theme-original/NavbarItem';
import { AuthContext } from '../../context/AuthContext';
import BrowserOnly from '@docusaurus/BrowserOnly';

// This is a wrapper for individual Navbar items.
// We check if the item is the 'Login' link and apply custom logic.
const CustomNavbarItem = (props) => {
  const { isLoggedIn } = useContext(AuthContext);

  // Apply custom logic only to the item that links to '/login'
  if (props.to === '/login') {
    // If the user is logged in, render nothing (hide the login button).
    if (isLoggedIn) {
      return null;
    }
  }

  // For all other navbar items, or for the login item when not logged in,
  // render the original Docusaurus NavbarItem component with its props.
  return <OriginalNavbarItem {...props} />;
};

// We wrap the component in BrowserOnly to ensure that `isLoggedIn` state
// (which relies on localStorage) is only checked on the client side.
// This prevents hydration mismatches between the server-rendered and client-rendered HTML.
export default function NavbarItem(props) {
    return (
        <BrowserOnly fallback={<OriginalNavbarItem {...props} />}>
            {() => <CustomNavbarItem {...props} />}
        </BrowserOnly>
    );
}
