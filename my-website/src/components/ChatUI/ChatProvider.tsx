import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatUI from './ChatUI';

const ChatProvider: React.FC = () => {
  const location = useLocation();
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(typeof window !== 'undefined');
  }, [location]);

  if (!isClient) return null; // Only render on client

  return <ChatUI />;
};

export default ChatProvider;