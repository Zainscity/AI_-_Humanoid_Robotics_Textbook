import React, { useContext } from 'react';
import Link from '@docusaurus/Link';
import { AuthContext } from '../../context/AuthContext';

const LoginNavbarItem = (props) => {
  const { isLoggedIn } = useContext(AuthContext);

  if (isLoggedIn) {
    return null; // Don't render anything if logged in
  }

  // Render a Docusaurus Link component directly
  return (
    <Link
      {...props} // Pass through any props from docusaurus.config.js
      to="/login"
      className="navbar__item navbar__link" // Apply default Docusaurus link styling
    >
      Login
    </Link>
  );
};

export default LoginNavbarItem;