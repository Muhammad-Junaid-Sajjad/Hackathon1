import React, { useState, useEffect, useRef } from 'react';
import Chatbot from '@site/src/components/Widgets';

interface TerminalLine {
  id: number;
  type: 'prompt' | 'command' | 'output' | 'success' | 'error';
  content: string;
}

const VscodeInterface: React.FC = () => {
  const [activePanel, setActivePanel] = useState<string>('welcome');
  const [terminalLines, setTerminalLines] = useState<TerminalLine[]>([]);
  const [showSignIn, setShowSignIn] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const [typedContent, setTypedContent] = useState('');
  const [codePanels] = useState([
    { id: 'welcome', title: 'Welcome.md', content: '# Welcome to Physical AI & Humanoid Robotics Textbook\n\n2026 Edition\n\nThis project was generated using Claude Code + Qwen + SpecifyPlus.' },
    { id: 'app', title: 'App.tsx', content: 'import React from \'react\';\n\nconst App = () => {\n  return (\n    <div>\n      <h1>Physical AI & Humanoid Robotics</h1>\n      <p>2026 Edition</p>\n    </div>\n  );\n};\n\nexport default App;' },
    { id: 'styles', title: 'styles.css', content: 'body {\n  background: #1e1e1e;\n  color: #d4d4d4;\n  font-family: var(--font-sans);\n}\n\n.container {\n  max-width: 1200px;\n  margin: 0 auto;\n  padding: 2rem;\n}' },
    { id: 'package', title: 'package.json', content: '{\n  "name": "physical-ai-textbook",\n  "version": "2026.0.0",\n  "description": "Physical AI & Humanoid Robotics Textbook",\n  "dependencies": {\n    "react": "^18.0.0",\n    "docusaurus": "^3.0.0"\n  }\n}' }
  ]);

  const terminalRef = useRef<HTMLDivElement>(null);

  // Advanced Terminal Animation
  useEffect(() => {
    const commands = [
      { type: 'prompt', content: '$ ' },
      { type: 'command', content: 'npm install' },
      { type: 'output', content: 'added 1492 packages from 843 contributors and audited 1493 packages in 12s' },
      { type: 'prompt', content: '$ ' },
      { type: 'command', content: 'npm run build' },
      { type: 'output', content: 'Creating an optimized production build...' },
      { type: 'output', content: 'Compiled successfully.' },
      { type: 'prompt', content: '$ ' },
      { type: 'command', content: 'ros2 launch humanoid_robot world_launch.py' },
      { type: 'output', content: '[INFO] [launch]: All nodes started [Controller, Vision, SensorFusion]' },
      { type: 'success', content: 'System Status: 2026 Edition Embodiment Ready!' }
    ];

    let index = 0;
    const interval = setInterval(() => {
      if (index < commands.length) {
        setTerminalLines(prev => [...prev, {
          id: Date.now() + index,
          type: commands[index].type as any,
          content: commands[index].content
        }]);
        index++;
      }
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  // Auto-scroll terminal
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [terminalLines]);

  // Typing effect when panel changes
  useEffect(() => {
    const currentPanel = codePanels.find(p => p.id === activePanel);
    if (!currentPanel) return;

    setIsTyping(true);
    setTypedContent('');
    let i = 0;
    const text = currentPanel.content;

    const typingInterval = setInterval(() => {
      if (i < text.length) {
        setTypedContent(prev => prev + text.charAt(i));
        i++;
      } else {
        setIsTyping(false);
        clearInterval(typingInterval);
      }
    }, 20);

    return () => clearInterval(typingInterval);
  }, [activePanel]);

  // Rotate code panels
  useEffect(() => {
    const panelIds = ['welcome', 'app', 'styles', 'package'];
    let currentIndex = 0;

    const interval = setInterval(() => {
      currentIndex = (currentIndex + 1) % panelIds.length;
      setActivePanel(panelIds[currentIndex]);
    }, 8000);

    return () => clearInterval(interval);
  }, []);

  const currentPanel = codePanels.find(panel => panel.id === activePanel) || codePanels[0];

  return (
    <div className="vscodeContainer" style={{ height: '85vh', minHeight: '800px', width: '100%' }}>
      <div className="vscodeHeader">
        <div className="vscodeControls">
          <div className="vscodeControl close"></div>
          <div className="vscodeControl minimize"></div>
          <div className="vscodeControl maximize"></div>
        </div>
        <div className="vscodeTitle">Frontier 2026 IDE - Physical AI Core</div>
      </div>

      <div className="vscodeContent">
        <div className="splitScreen">
          <div className="sidebarSplit" style={{ width: '200px' }}>
            <div className="fileExplorer">
              <h4>EXPLORER</h4>
              <ul className="fileList">
                {codePanels.map(panel => (
                  <li
                    key={panel.id}
                    className={activePanel === panel.id ? 'active' : ''}
                    onClick={() => { setActivePanel(panel.id); setShowSignIn(false); }}
                  >
                    {panel.title}
                  </li>
                ))}
                <li onClick={() => setShowSignIn(true)} className={showSignIn ? 'active' : ''}>Sign In</li>
              </ul>
            </div>
          </div>

          <div className="contentSplit" style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
            <div className="editorSplit" style={{ flex: 1, background: '#1e1e1e', padding: '1.5rem', overflow: 'auto' }}>
              <pre className="codeEditor" style={{ margin: 0, whiteSpace: 'pre-wrap' }}>
                <code style={{ color: '#d4d4d4', fontFamily: 'var(--font-mono)' }}>
                  {typedContent}
                  {isTyping && <span className="terminalTyping">|</span>}
                </code>
              </pre>
            </div>

            <div className="terminalSplit" ref={terminalRef} style={{ height: '250px', background: '#0c0c0c', padding: '1rem', borderTop: '2px solid #333' }}>
              {terminalLines.map((line) => (
                <div key={line.id} className={`terminalLine terminal${line.type.charAt(0).toUpperCase() + line.type.slice(1)}`}>
                  {line.type === 'prompt' && <span className="terminalPrompt" style={{ color: '#4ec9b0' }}>{line.content}</span>}
                  {(line.type === 'command' || line.type === 'output') && <span className="terminalCommand" style={{ color: '#d4d4d4' }}>{line.content}</span>}
                  {line.type === 'success' && <span className="terminalSuccess" style={{ color: '#4ec9b0' }}>{line.content}</span>}
                  {line.type === 'error' && <span className="terminalError" style={{ color: '#f48771' }}>{line.content}</span>}
                </div>
              ))}
            </div>
          </div>
        </div>

        <div className="chatbotContainer" style={{ zIndex: 100 }}>
          <div className="chatHeader">2026 RAG Assistant</div>
          <div className="chatMessages">
            <Chatbot apiUrl="" />
          </div>
        </div>
      </div>
    </div>
  );
};

export default VscodeInterface;