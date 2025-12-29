/**
 * DocItem Layout Wrapper
 *
 * Adds ChapterTools (Personalization & Translation buttons) to each chapter.
 *
 * Requirements addressed:
 * - Requirement 6: Chapter personalization for logged users
 * - Requirement 7: Urdu translation button for chapters
 */

import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import ChapterTools from '@site/src/components/ChapterTools';
import { useAuth } from '@site/src/components/Auth/AuthContext';

type Props = WrapperProps<typeof LayoutType>;

// API URL from environment
const API_URL = typeof window !== 'undefined'
  ? (window as any).__ENV__?.API_URL || ''
  : '';

export default function LayoutWrapper(props: Props): JSX.Element {
  const { metadata, contentRef } = useDoc();
  const { user, isLoggedIn } = useAuth();

  // Extract chapter ID from the document path (e.g., "M2/C1/S3" from "M2-C1-S3")
  const chapterId = metadata.id || 'unknown';

  // Get the content element for extracting text
  const [chapterContent, setChapterContent] = React.useState('');

  React.useEffect(() => {
    // Extract text content from the rendered document
    if (contentRef?.current) {
      const textContent = contentRef.current.innerText || '';
      setChapterContent(textContent);
    }
  }, [contentRef]);

  return (
    <>
      {/* ChapterTools at the top of each doc */}
      <ChapterTools
        chapterId={chapterId}
        chapterContent={chapterContent}
        isLoggedIn={isLoggedIn}
        userId={user?.id}
        apiUrl={API_URL}
      />
      {/* Original doc content */}
      <Layout {...props} />
    </>
  );
}
