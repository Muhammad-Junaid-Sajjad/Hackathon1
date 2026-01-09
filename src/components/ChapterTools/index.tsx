/**
 * Chapter Tools Component
 *
 * Provides personalization and translation buttons at the top of each chapter.
 *
 * Requirements addressed:
 * - Requirement 6: Chapter personalization for logged users
 * - Requirement 7: Urdu translation for logged users
 */

import React, { useState, useCallback, useMemo } from 'react';
import DOMPurify from 'dompurify';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

/**
 * Sanitize HTML content to prevent XSS attacks.
 * Uses DOMPurify library for robust sanitization.
 *
 * SECURITY: This is critical for any content rendered via dangerouslySetInnerHTML.
 * DOMPurify is the industry-standard library for HTML sanitization.
 */
function sanitizeHtml(html: string): string {
  if (typeof window === 'undefined') {
    // Server-side: strip all HTML tags for safety
    return html.replace(/<[^>]*>/g, '');
  }
  return DOMPurify.sanitize(html, {
    ALLOWED_TAGS: [
      'p', 'br', 'strong', 'em', 'code', 'pre', 'ul', 'ol', 'li',
      'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'a', 'blockquote',
      'table', 'thead', 'tbody', 'tr', 'th', 'td', 'span', 'div'
    ],
    ALLOWED_ATTR: ['href', 'class', 'id', 'dir', 'lang'],
    ALLOW_DATA_ATTR: false,
  });
}

interface ChapterToolsProps {
  chapterId: string;
  chapterContent: string;
  isLoggedIn: boolean;
  userId?: string;
  apiUrl?: string;
}

type ActiveTool = 'none' | 'personalizing' | 'translating' | 'personalized' | 'translated';

const ChapterTools: React.FC<ChapterToolsProps> = ({
  chapterId,
  chapterContent,
  isLoggedIn,
  userId,
  apiUrl = ''
}) => {
  const [activeTool, setActiveTool] = useState<ActiveTool>('none');
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = useCallback(async () => {
    if (!isLoggedIn) {
      setError('Please sign in to personalize content');
      return;
    }

    setActiveTool('personalizing');
    setError(null);

    try {
      const response = await fetch(`${apiUrl}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_content: chapterContent,
          user_id: userId,
          chapter_id: chapterId,
        }),
      });

      if (!response.ok) throw new Error('Personalization failed');

      const data = await response.json();
      setPersonalizedContent(data.personalized_content);
      setActiveTool('personalized');
    } catch (err) {
      setError('Failed to personalize content. Please try again.');
      setActiveTool('none');
    }
  }, [isLoggedIn, userId, chapterId, chapterContent, apiUrl]);

  const handleTranslate = useCallback(async () => {
    if (!isLoggedIn) {
      setError('Please sign in to translate content');
      return;
    }

    setActiveTool('translating');
    setError(null);

    try {
      const response = await fetch(`${apiUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: chapterContent,
          chapter_id: chapterId,
        }),
      });

      if (!response.ok) throw new Error('Translation failed');

      const data = await response.json();
      setTranslatedContent(data.translated_content);
      setActiveTool('translated');
    } catch (err) {
      setError('Failed to translate content. Please try again.');
      setActiveTool('none');
    }
  }, [isLoggedIn, chapterId, chapterContent, apiUrl]);

  const handleReset = useCallback(() => {
    setActiveTool('none');
    setPersonalizedContent(null);
    setTranslatedContent(null);
    setError(null);
  }, []);

  return (
    <div className={styles.chapterTools}>
      {/* Tool Buttons */}
      <div className={styles.toolButtons}>
        {/* Left side buttons */}
        <div className={styles.leftButtons}>
          {/* Personalize Button */}
          <button
            onClick={handlePersonalize}
            disabled={activeTool === 'personalizing' || activeTool === 'translating'}
            className={`${styles.toolButton} ${activeTool === 'personalized' ? styles.active : ''}`}
            title={isLoggedIn ? 'Personalize content based on your background' : 'Sign in to personalize'}
          >
            {activeTool === 'personalizing' ? (
              <>
                <span className={styles.spinner}></span>
                Personalizing...
              </>
            ) : (
              <>
                <span className={styles.icon}>✨</span>
                {activeTool === 'personalized' ? 'Personalized' : 'Personalize for Me'}
              </>
            )}
          </button>

          {/* Translate Button */}
          <button
            onClick={handleTranslate}
            disabled={activeTool === 'personalizing' || activeTool === 'translating'}
            className={`${styles.toolButton} ${styles.translateButton} ${activeTool === 'translated' ? styles.active : ''}`}
            title={isLoggedIn ? 'Translate content to Urdu' : 'Sign in to translate'}
          >
            {activeTool === 'translating' ? (
              <>
                <span className={styles.spinner}></span>
                Translating...
              </>
            ) : (
              <>
                <span className={styles.icon}>🌐</span>
                {activeTool === 'translated' ? 'اردو میں' : 'Translate to Urdu'}
              </>
            )}
          </button>

          {/* Reset Button */}
          {(activeTool === 'personalized' || activeTool === 'translated') && (
            <button
              onClick={handleReset}
              className={`${styles.toolButton} ${styles.resetButton}`}
            >
              <span className={styles.icon}>↩️</span>
              Show Original
            </button>
          )}
        </div>

        {/* Right side buttons */}
        <div className={styles.rightButtons}>
          {/* External Links Button */}
          <Link to="/#external-resources" className={`${styles.toolButton} ${styles.externalLinksButton}`} title="View External Learning Resources">
            <span className={styles.icon}>📚</span>
            External Links
          </Link>

          {/* Home Button */}
          <Link to="/" className={`${styles.toolButton} ${styles.homeButton}`} title="Return to Homepage">
            <span className={styles.icon}>🏠</span>
            Home
          </Link>
        </div>
      </div>

      {/* Login Prompt */}
      {!isLoggedIn && (
        <div className={styles.loginPrompt}>
          <span>🔒</span>
          <a href="/auth/signin">Sign in</a> to unlock personalization and translation features
        </div>
      )}

      {/* Error Message */}
      {error && (
        <div className={styles.errorMessage}>
          <span>⚠️</span>
          {error}
        </div>
      )}

      {/* Personalization Info */}
      {activeTool === 'personalized' && (
        <div className={styles.infoMessage}>
          <span>✨</span>
          Content has been personalized based on your background.
          <strong> Adaptations include additional explanations and tips tailored to your experience level.</strong>
        </div>
      )}

      {/* Translation Info */}
      {activeTool === 'translated' && (
        <div className={styles.infoMessage}>
          <span>🌐</span>
          مواد کا اردو میں ترجمہ کیا گیا ہے۔ / Content has been translated to Urdu.
        </div>
      )}

      {/* Modified Content Display */}
      {(personalizedContent || translatedContent) && (
        <div className={styles.modifiedContent}>
          <div
            className={styles.contentArea}
            dangerouslySetInnerHTML={{
              __html: sanitizeHtml(
                activeTool === 'translated'
                  ? translatedContent || ''
                  : personalizedContent || ''
              )
            }}
          />
        </div>
      )}
    </div>
  );
};

export default ChapterTools;
