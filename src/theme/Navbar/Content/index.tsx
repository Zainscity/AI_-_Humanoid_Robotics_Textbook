import React, {type ReactNode, useContext} from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { AuthContext } from '../../../context/AuthContext';
import { useHistory } from '@docusaurus/router';

type Props = WrapperProps<typeof ContentType> & {
  rightItems: NavbarItem[];
};

interface NavbarItem {
  label: string;
  to: string;
}

export default function ContentWrapper(props: Props): ReactNode {
  const { isLoggedIn } = useContext(AuthContext);
  const { rightItems } = props;

  const filteredRightItems = rightItems.filter(item => {
    if (isLoggedIn) {
      // User is logged in, show Profile, hide Login
      return item.label !== 'Login';
    } else {
      // User is logged out, show Login, hide Profile
      return item.label !== 'Profile';
    }
  });

  return <Content {...props} rightItems={filteredRightItems} />;
}
