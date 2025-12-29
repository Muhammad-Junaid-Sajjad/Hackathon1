# Docusaurus Component Generator

Generate React components for the Physical AI Textbook Docusaurus site following project patterns.

## Arguments
- `$ARGUMENTS` - Component name and type: "ComponentName:type" (types: page, widget, feature, layout)

## Instructions

You are the **Docusaurus Component Generator**. Create high-quality React components that integrate seamlessly with the Physical AI & Humanoid Robotics textbook.

### Project Component Standards

Based on analysis of existing components:

1. **TypeScript** - All components use `.tsx` extension
2. **CSS Modules** - Styles in `styles.module.css` alongside component
3. **Docusaurus Theming** - Use `var(--ifm-*)` CSS variables
4. **Dark Mode** - Support `[data-theme='dark']` selectors
5. **Mobile Responsive** - Include `@media` queries for mobile

### Step 1: Parse Arguments

```
Input: "QuizWidget:widget"
Output: { name: "QuizWidget", type: "widget" }

Input: "AuthPage:page"
Output: { name: "AuthPage", type: "page" }
```

### Step 2: Component Type Templates

#### Type: `widget` (Small, reusable UI elements)

```typescript
// src/components/{Name}/index.tsx
/**
 * {Name} Widget Component
 *
 * {Brief description}
 */

import React, { useState } from 'react';
import styles from './styles.module.css';

interface {Name}Props {
  // Props interface
}

const {Name}: React.FC<{Name}Props> = (props) => {
  // Component implementation
  return (
    <div className={styles.container}>
      {/* Widget content */}
    </div>
  );
};

export default {Name};
```

#### Type: `feature` (Larger feature components like Chatbot, ChapterTools)

```typescript
// src/components/{Name}/index.tsx
/**
 * {Name} Feature Component
 *
 * Requirements addressed:
 * - [Requirement details]
 */

import React, { useState, useCallback, useEffect } from 'react';
import styles from './styles.module.css';

interface {Name}Props {
  apiUrl?: string;
  isLoggedIn?: boolean;
  userId?: string;
  // Feature-specific props
}

const {Name}: React.FC<{Name}Props> = ({
  apiUrl = '',
  isLoggedIn = false,
  userId,
}) => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Feature implementation with API calls
  const handleAction = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch(`${apiUrl}/api/endpoint`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ /* data */ }),
      });
      if (!response.ok) throw new Error('Request failed');
      const data = await response.json();
      // Handle response
    } catch (err: any) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  }, [apiUrl]);

  return (
    <div className={styles.featureContainer}>
      {error && <div className={styles.errorMessage}>{error}</div>}
      {loading && <div className={styles.loadingSpinner} />}
      {/* Feature content */}
    </div>
  );
};

export default {Name};
```

#### Type: `page` (Full page components)

```typescript
// src/pages/{name}/index.tsx
/**
 * {Name} Page
 *
 * Route: /{name}
 */

import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css';

export default function {Name}Page(): JSX.Element {
  return (
    <Layout
      title="{Name}"
      description="{Page description}"
    >
      <main className={styles.pageContainer}>
        <div className="container">
          <h1>{Name}</h1>
          {/* Page content */}
        </div>
      </main>
    </Layout>
  );
}
```

#### Type: `layout` (Theme/Layout components)

```typescript
// src/theme/{Name}/index.tsx
/**
 * Custom {Name} Layout Component
 *
 * Wraps Docusaurus theme component with additional features.
 */

import React from 'react';
import OriginalComponent from '@theme-original/{Name}';
import type {Name}Type from '@theme/{Name}';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof {Name}Type>;

export default function {Name}Wrapper(props: Props): JSX.Element {
  return (
    <>
      <OriginalComponent {...props} />
      {/* Additional elements */}
    </>
  );
}
```

### Step 3: CSS Module Template

```css
/* src/components/{Name}/styles.module.css */

/* Container */
.container {
  padding: 16px;
  border-radius: 8px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-200);
}

/* Loading states */
.loadingSpinner {
  width: 24px;
  height: 24px;
  border: 3px solid var(--ifm-color-emphasis-300);
  border-top-color: var(--ifm-color-primary);
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* Error states */
.errorMessage {
  padding: 12px 16px;
  background: #fee2e2;
  color: #991b1b;
  border-radius: 8px;
  font-size: 14px;
}

[data-theme='dark'] .errorMessage {
  background: #7f1d1d;
  color: #fecaca;
}

/* Success states */
.successMessage {
  padding: 12px 16px;
  background: #dcfce7;
  color: #166534;
  border-radius: 8px;
}

[data-theme='dark'] .successMessage {
  background: #14532d;
  color: #bbf7d0;
}

/* Buttons */
.primaryButton {
  padding: 10px 20px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 8px;
  font-weight: 600;
  cursor: pointer;
  transition: background 0.2s;
}

.primaryButton:hover:not(:disabled) {
  background: var(--ifm-color-primary-dark);
}

.primaryButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.secondaryButton {
  padding: 10px 20px;
  background: transparent;
  color: var(--ifm-color-primary);
  border: 2px solid var(--ifm-color-primary);
  border-radius: 8px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.secondaryButton:hover:not(:disabled) {
  background: var(--ifm-color-primary);
  color: white;
}

/* Responsive */
@media (max-width: 768px) {
  .container {
    padding: 12px;
  }
}
```

### Step 4: Integration Points

For components that need to be integrated into Docusaurus:

#### Adding to DocItem (for chapter-level components)
```typescript
// src/theme/DocItem/Layout/index.tsx
import ChapterTools from '@site/src/components/ChapterTools';

// In the render:
<ChapterTools
  chapterId={doc.id}
  chapterContent={doc.content}
  isLoggedIn={/* from auth context */}
/>
```

#### Adding to Root (for global components like Chatbot)
```typescript
// src/theme/Root.tsx
import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot apiUrl={process.env.API_URL || ''} />
    </>
  );
}
```

### Step 5: Generate Component

Based on the arguments, create:
1. Component file: `src/components/{Name}/index.tsx`
2. Styles file: `src/components/{Name}/styles.module.css`
3. Integration code (if needed)

### Step 6: Validation Checklist

- [ ] TypeScript compiles without errors
- [ ] CSS uses Docusaurus variables
- [ ] Dark mode supported
- [ ] Mobile responsive
- [ ] Loading/error states handled
- [ ] Props documented with TypeScript interfaces
- [ ] Follows existing component patterns

### Step 7: Report

```
╔══════════════════════════════════════════════════════════════╗
║              COMPONENT GENERATION REPORT                      ║
╠══════════════════════════════════════════════════════════════╣
║ Component: {Name}                                             ║
║ Type: {type}                                                  ║
╠══════════════════════════════════════════════════════════════╣
║ Files Created:                                                ║
║ ├─ src/components/{Name}/index.tsx                            ║
║ └─ src/components/{Name}/styles.module.css                    ║
╠══════════════════════════════════════════════════════════════╣
║ Features:                                                     ║
║ ├─ TypeScript: ✅                                             ║
║ ├─ CSS Modules: ✅                                            ║
║ ├─ Dark Mode: ✅                                              ║
║ ├─ Responsive: ✅                                             ║
║ └─ Error Handling: ✅                                         ║
╠══════════════════════════════════════════════════════════════╣
║ Integration:                                                  ║
║ [Instructions for integrating the component]                  ║
╚══════════════════════════════════════════════════════════════╝

Usage:
import {Name} from '@site/src/components/{Name}';

<{Name} prop1="value" prop2={123} />
```

## Reusability

- `/generate-component QuizWidget:widget` - Create quiz widget
- `/generate-component AuthPage:page` - Create auth page
- `/generate-component ProgressTracker:feature` - Create feature component
- `/generate-component DocItemWrapper:layout` - Create theme wrapper

Follows project patterns discovered in existing components.
