/**
 * RAG Chatbot Component for Physical AI & Humanoid Robotics Textbook
 *
 * Requirements addressed:
 * - Requirement 2: Embedded RAG chatbot with OpenAI/Qdrant
 * - Supports answering questions about selected text
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    content: string;
    source: string;
    score: number;
  }>;
}

interface ChatbotProps {
  apiUrl?: string;
  userId?: string;
}

const Chatbot: React.FC<ChatbotProps> = ({
  apiUrl = '/api/chat',
  userId
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection on the page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async () => {
    if (!input.trim() && !selectedText) return;

    const userMessage: ChatMessage = {
      role: 'user',
      content: input.trim()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: input.trim(),
          selected_text: selectedText,
          conversation_history: messages.slice(-6).map(m => ({
            role: m.role,
            content: m.content
          })),
          user_id: userId
        }),
      });

      if (!response.ok) throw new Error('Failed to get response');

      const data = await response.json();

      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(null); // Clear selected text after using it
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.'
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText(null);
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerInfo}>
              <span className={styles.headerIcon}>🤖</span>
              <div>
                <h3>AI Tutor</h3>
                <span className={styles.headerSubtitle}>Physical AI & Robotics</span>
              </div>
            </div>
            <button onClick={clearChat} className={styles.clearButton} title="Clear chat">
              🗑️
            </button>
          </div>

          {/* Selected Text Indicator */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <span>📝 Asking about selected text ({selectedText.length} chars)</span>
              <button onClick={() => setSelectedText(null)}>✕</button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <h4>👋 Welcome to the AI Tutor!</h4>
                <p>Ask me anything about the Physical AI & Humanoid Robotics textbook.</p>
                <div className={styles.suggestions}>
                  <button onClick={() => setInput('Explain ROS 2 topics and services')}>
                    Explain ROS 2 topics
                  </button>
                  <button onClick={() => setInput('How does ZMP balance control work?')}>
                    ZMP balance control
                  </button>
                  <button onClick={() => setInput('What is domain randomization?')}>
                    Domain randomization
                  </button>
                </div>
                <p className={styles.tip}>
                  💡 <strong>Tip:</strong> Select text on any page, then ask a question about it!
                </p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${styles[msg.role]}`}
              >
                <div className={styles.messageContent}>
                  {msg.role === 'assistant' ? (
                    <div dangerouslySetInnerHTML={{ __html: formatMessage(msg.content) }} />
                  ) : (
                    msg.content
                  )}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <details>
                      <summary>📚 Sources ({msg.sources.length})</summary>
                      <ul>
                        {msg.sources.map((source, sIdx) => (
                          <li key={sIdx}>
                            <span className={styles.sourceScore}>
                              {Math.round(source.score * 100)}%
                            </span>
                            {source.content}
                          </li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask a question..."}
              rows={1}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || (!input.trim() && !selectedText)}
              className={styles.sendButton}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

/**
 * Safe markdown-like formatting with XSS protection.
 *
 * SECURITY: All user-generated content is escaped before rendering.
 * Only safe formatting tags are allowed.
 */
function escapeHtml(text: string): string {
  const htmlEscapes: Record<string, string> = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;',
  };
  return text.replace(/[&<>"']/g, (char) => htmlEscapes[char]);
}

function formatMessage(text: string): string {
  // First escape HTML to prevent XSS
  let safeText = escapeHtml(text);

  // Then apply safe formatting (after escaping, so no injection possible)
  return safeText
    // Code blocks (restore < and > inside code only)
    .replace(/```(\w+)?\n([\s\S]*?)```/g, (_, lang, code) =>
      `<pre><code>${code}</code></pre>`)
    // Inline code
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    // Bold
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    // Italic
    .replace(/\*([^*]+)\*/g, '<em>$1</em>')
    // Line breaks
    .replace(/\n/g, '<br/>');
}

export default Chatbot;
