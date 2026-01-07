import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './DocumentationEnhancements.module.css';

interface DocumentationEnhancementsProps {
  children?: React.ReactNode;
  hasFloatingToc?: boolean;
}

const DocumentationEnhancements: React.FC<DocumentationEnhancementsProps> = ({
  children,
  hasFloatingToc = true,
}) => {
  const [showFloatingToc, setShowFloatingToc] = useState(false);
  const [tocItems, setTocItems] = useState<Array<{id: string, text: string, level: number}>>([]);
  const [activeId, setActiveId] = useState('');

  // Extract table of contents from headings
  useEffect(() => {
    const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
    const items = Array.from(headings).map(heading => {
      const id = heading.getAttribute('id');
      const text = heading.textContent || '';
      const level = parseInt(heading.tagName.charAt(1));
      return id ? { id, text, level } : null;
    }).filter(Boolean) as Array<{id: string, text: string, level: number}>;

    setTocItems(items);

    // Handle scroll to update active TOC item
    const handleScroll = () => {
      const scrollPosition = window.scrollY + 200; // Offset to account for navbar

      for (let i = items.length - 1; i >= 0; i--) {
        const element = document.getElementById(items[i].id);
        if (element && element.offsetTop <= scrollPosition) {
          setActiveId(items[i].id);
          break;
        }
      }
    };

    window.addEventListener('scroll', handleScroll);
    handleScroll(); // Initial check

    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  // Scroll to heading function
  const scrollToHeading = (id: string) => {
    const element = document.getElementById(id);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth' });
      setActiveId(id);
    }
  };

  return (
    <div className={styles.documentationContainer}>
      {children}

      {/* Floating Table of Contents */}
      {hasFloatingToc && tocItems.length > 0 && (
        <div className={clsx(styles.floatingToc, showFloatingToc ? styles.visible : '')}>
          <button
            className={styles.tocToggle}
            onClick={() => setShowFloatingToc(!showFloatingToc)}
            aria-label={showFloatingToc ? "Hide Table of Contents" : "Show Table of Contents"}
          >
            {showFloatingToc ? '✕' : '☰'}
          </button>

          {showFloatingToc && (
            <div className={styles.tocContent}>
              <h3 className={styles.tocTitle}>Contents</h3>
              <ul className={styles.tocList}>
                {tocItems.map((item, index) => (
                  <li
                    key={index}
                    className={clsx(
                      styles.tocItem,
                      styles[`level${item.level}`],
                      activeId === item.id && styles.active
                    )}
                    style={{ paddingLeft: `${(item.level - 1) * 1}rem` }}
                  >
                    <button
                      className={styles.tocLink}
                      onClick={() => scrollToHeading(item.id)}
                    >
                      {item.text}
                    </button>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default DocumentationEnhancements;