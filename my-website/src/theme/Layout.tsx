import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatUI from '../components/ChatUI';
import { useEffect } from 'react';

type LayoutProps = {
  children: React.ReactNode;
};

export default function Layout(props: LayoutProps): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatUI />
    </>
  );
}