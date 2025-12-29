/**
 * Root Theme Wrapper
 *
 * Wraps the entire Docusaurus app with:
 * - AuthProvider for authentication state
 * - Chatbot widget for RAG-powered Q&A
 *
 * This component is automatically loaded by Docusaurus.
 */

import React from 'react';
import { AuthProvider } from '@site/src/components/Auth/AuthContext';
import Chatbot from '@site/src/components/Chatbot';

// API URL from environment or default for development
const API_URL = typeof window !== 'undefined'
  ? (window as any).__ENV__?.API_URL || ''
  : '';

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  return (
    <AuthProvider apiUrl={API_URL}>
      {children}
      {/* RAG Chatbot - appears on all pages */}
      <Chatbot apiUrl={API_URL} />
    </AuthProvider>
  );
}
