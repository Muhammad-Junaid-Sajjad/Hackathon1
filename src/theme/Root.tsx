import React, { useState, useEffect } from 'react';
import { AuthProvider } from '@site/src/auth/AuthProvider';

export default function Root({ children }) {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  if (!isClient) {
    // During SSR, return children without AuthProvider
    return <>{children}</>;
  }

  // On client side, wrap with AuthProvider
  return <AuthProvider>{children}</AuthProvider>;
}