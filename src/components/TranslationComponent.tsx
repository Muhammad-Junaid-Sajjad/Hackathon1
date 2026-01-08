import React, { useState } from 'react';
import { useAuth } from '@site/src/auth/AuthProvider';

interface TranslationComponentProps {
  content: string;
  chapterId: string;
}

const TranslationComponent: React.FC<TranslationComponentProps> = ({ content, chapterId }) => {
  const { isAuthenticated } = useAuth();
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    if (!isAuthenticated) {
      alert('Please log in to access translation features');
      return;
    }

    setIsTranslating(true);
    setError(null);

    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          chapter_id: chapterId
        }),
      });

      if (!response.ok) {
        throw new Error(`Translation API error: ${response.status}`);
      }

      const data = await response.json();
      setTranslatedContent(data.translated_content);
    } catch (err) {
      console.error('Translation error:', err);
      setError('Failed to translate content. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const resetTranslation = () => {
    setTranslatedContent(null);
    setError(null);
  };

  return (
    <div className="translation-component">
      {!translatedContent ? (
        <div>
          <button 
            onClick={handleTranslate} 
            disabled={isTranslating}
            className={`button ${isTranslating ? 'button--disabled' : 'button--secondary'}`}
          >
            {isTranslating ? 'Translating...' : '.Translate to Urdu →'}
          </button>
          {error && <div className="alert alert--danger margin-top--sm">{error}</div>}
        </div>
      ) : (
        <div>
          <div className="alert alert--success margin-bottom--sm">
            <strong>Urdu Translation</strong>
            <button 
              onClick={resetTranslation} 
              className="float-right button button--sm button--outline button--primary"
            >
              Show Original
            </button>
          </div>
          <div className="urdu-content" dir="rtl" style={{ lineHeight: '2', fontSize: '1.1em' }}>
            {translatedContent.split('\n').map((paragraph, i) => (
              <p key={i}>{paragraph}</p>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default TranslationComponent;