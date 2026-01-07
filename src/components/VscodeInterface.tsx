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
  const [codePanels] = useState([
    { id: 'welcome', title: 'Welcome.md', content: '# Welcome to Physical AI & Humanoid Robotics Textbook\n\n2026 Edition\n\nThis project was generated using Claude Code + Qwen + SpecifyPlus.' },
    { id: 'app', title: 'App.tsx', content: 'import React from \'react\';\n\nconst App = () => {\n  return (\n    <div>\n      <h1>Physical AI & Humanoid Robotics</h1>\n      <p>2026 Edition</p>\n    </div>\n  );\n};\n\nexport default App;' },
    { id: 'styles', title: 'styles.css', content: 'body {\n  background: #1e1e1e;\n  color: #d4d4d4;\n  font-family: var(--font-sans);\n}\n\n.container {\n  max-width: 1200px;\n  margin: 0 auto;\n  padding: 2rem;\n}' },
    { id: 'package', title: 'package.json', content: '{\n  "name": "physical-ai-textbook",\n  "version": "2026.0.0",\n  "description": "Physical AI & Humanoid Robotics Textbook",\n  "dependencies": {\n    "react": "^18.0.0",\n    "docusaurus": "^3.0.0"\n  }\n}' }
  ]);

  const terminalRef = useRef<HTMLDivElement>(null);

  // Simulate terminal output
  useEffect(() => {
    const commands = [
      { type: 'prompt', content: '$ ' },
      { type: 'command', content: 'Building website through Qwen/Claude Code and SpecifyPlus...' },
      { type: 'output', content: 'Compiling TypeScript files...' },
      { type: 'output', content: 'Processing assets...' },
      { type: 'output', content: 'Generating documentation...' },
      { type: 'success', content: 'Project generated successfully!' },
      { type: 'output', content: 'Entire website and project completed from 1 prompt' },
      { type: 'success', content: 'Deployment ready!' }
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
      } else {
        clearInterval(interval);
      }
    }, 1500);

    return () => clearInterval(interval);
  }, []);

  // Auto-scroll terminal
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [terminalLines]);

  // Rotate code panels
  useEffect(() => {
    const panelIds = ['welcome', 'app', 'styles', 'package'];
    let currentIndex = 0;

    const interval = setInterval(() => {
      currentIndex = (currentIndex + 1) % panelIds.length;
      setActivePanel(panelIds[currentIndex]);
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  const currentPanel = codePanels.find(panel => panel.id === activePanel) || codePanels[0];

  return (
    <div className="vscodeContainer">
      <div className="vscodeHeader">
        <div className="vscodeControls">
          <div className="vscodeControl close"></div>
          <div className="vscodeControl minimize"></div>
          <div className="vscodeControl maximize"></div>
        </div>
        <div className="vscodeTitle">Physical AI & Humanoid Robotics Textbook - 2026 Edition</div>
      </div>

      <div className="vscodeContent">
        <div className="splitScreen">
          <div className="sidebarSplit">
            <div className="fileExplorer">
              <h4>EXPLORER</h4>
              <ul className="fileList">
                <li onClick={() => setShowSignIn(false)}>Welcome.md</li>
                <li onClick={() => setShowSignIn(false)}>App.tsx</li>
                <li onClick={() => setShowSignIn(false)}>styles.css</li>
                <li onClick={() => setShowSignIn(false)}>package.json</li>
                <li onClick={() => setShowSignIn(true)} className={showSignIn ? 'active' : ''}>Sign In</li>
              </ul>
            </div>

            <div className="fileExplorer">
              <h4>PROJECT</h4>
              <ul className="fileList">
                <li>src/</li>
                <li>docs/</li>
                <li>public/</li>
                <li>package.json</li>
                <li>README.md</li>
              </ul>
            </div>
          </div>

          <div className="contentSplit">
            {showSignIn ? (
              <div className="signInPage">
                <div className="signInContainer">
                  <h2 className="signInTitle">Sign In to Your Account</h2>
                  <form className="signInForm">
                    <div className="formGroup">
                      <label htmlFor="email">Email</label>
                      <input type="email" id="email" placeholder="Enter your email" />
                    </div>
                    <div className="formGroup">
                      <label htmlFor="password">Password</label>
                      <input type="password" id="password" placeholder="Enter your password" />
                    </div>
                    <button type="submit" className="signInButton">Sign In</button>
                  </form>
                  <p style={{ marginTop: '1rem', fontSize: '0.9rem', color: '#999' }}>
                    Don't have an account? <a href="/auth/signup" style={{ color: '#007acc' }}>Sign Up</a>
                  </p>
                </div>
              </div>
            ) : (
              <>
                <div className="editorSplit">
                  <pre className="codeEditor">
                    <code>{currentPanel.content}</code>
                  </pre>
                </div>

                <div className="terminalSplit" ref={terminalRef}>
                  {terminalLines.map((line) => (
                    <div key={line.id} className={`terminalLine terminal${line.type.charAt(0).toUpperCase() + line.type.slice(1)}`}>
                      {line.type === 'prompt' && <span className="terminalPrompt">{line.content}</span>}
                      {(line.type === 'command' || line.type === 'output') && <span className="terminalCommand">{line.content}</span>}
                      {line.type === 'success' && <span className="terminalSuccess">{line.content}</span>}
                      {line.type === 'error' && <span className="terminalError">{line.content}</span>}
                    </div>
                  ))}
                </div>
              </>
            )}
          </div>
        </div>

        {/* Chatbot positioned in the bottom right corner */}
        <div className="chatbotContainer">
          <div className="chatHeader">AI Assistant</div>
          <div className="chatMessages">
            <Chatbot apiUrl="" />
          </div>
        </div>
      </div>
    </div>
  );
};

export default VscodeInterface;