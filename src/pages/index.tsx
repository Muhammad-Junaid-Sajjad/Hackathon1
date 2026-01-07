/**
 * Homepage - Physical AI & Humanoid Robotics Textbook
 *
 * Design inspired by: Figure.ai (futuristic robotics), GitHub (clean), Panaversity (educational)
 */

import React, { useEffect, useState, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import VscodeInterface from '@site/src/components/VscodeInterface';

// Animated Robot SVG Component
function RobotIllustration() {
  return (
    <div className={styles.robotContainer}>
      <svg
        viewBox="0 0 400 400"
        className={styles.robotSvg}
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Glowing background circle */}
        <circle cx="200" cy="200" r="180" fill="url(#glow)" className={styles.glowCircle} />

        {/* Robot head */}
        <rect x="130" y="80" width="140" height="120" rx="20" fill="url(#headGradient)" className={styles.robotHead} />

        {/* Eyes */}
        <circle cx="165" cy="130" r="18" fill="#0a0a0f" />
        <circle cx="235" cy="130" r="18" fill="#0a0a0f" />
        <circle cx="165" cy="130" r="10" fill="#818cf8" className={styles.eyeGlow} />
        <circle cx="235" cy="130" r="10" fill="#818cf8" className={styles.eyeGlow} />

        {/* Antenna */}
        <line x1="200" y1="80" x2="200" y2="50" stroke="url(#antennaGradient)" strokeWidth="4" />
        <circle cx="200" cy="45" r="8" fill="#818cf8" className={styles.antennaLight} />

        {/* Body */}
        <rect x="120" y="210" width="160" height="100" rx="15" fill="url(#bodyGradient)" />

        {/* Chest display */}
        <rect x="150" y="230" width="100" height="50" rx="8" fill="#0a0a0f" />
        <rect x="160" y="240" width="80" height="4" rx="2" fill="#818cf8" className={styles.displayLine} />
        <rect x="160" y="250" width="60" height="4" rx="2" fill="#6366f1" className={styles.displayLine} />
        <rect x="160" y="260" width="70" height="4" rx="2" fill="#818cf8" className={styles.displayLine} />

        {/* Arms */}
        <rect x="70" y="220" width="40" height="80" rx="10" fill="url(#armGradient)" className={styles.leftArm} />
        <rect x="290" y="220" width="40" height="80" rx="10" fill="url(#armGradient)" className={styles.rightArm} />

        {/* Legs */}
        <rect x="140" y="320" width="35" height="60" rx="8" fill="url(#legGradient)" />
        <rect x="225" y="320" width="35" height="60" rx="8" fill="url(#legGradient)" />

        {/* Gradients */}
        <defs>
          <radialGradient id="glow" cx="0.5" cy="0.5" r="0.5">
            <stop offset="0%" stopColor="#818cf8" stopOpacity="0.3" />
            <stop offset="100%" stopColor="#818cf8" stopOpacity="0" />
          </radialGradient>
          <linearGradient id="headGradient" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stopColor="#4f46e5" />
            <stop offset="100%" stopColor="#7c3aed" />
          </linearGradient>
          <linearGradient id="bodyGradient" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stopColor="#3730a3" />
            <stop offset="100%" stopColor="#5b21b6" />
          </linearGradient>
          <linearGradient id="armGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#4338ca" />
            <stop offset="100%" stopColor="#6366f1" />
          </linearGradient>
          <linearGradient id="legGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#3730a3" />
            <stop offset="100%" stopColor="#4f46e5" />
          </linearGradient>
          <linearGradient id="antennaGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#818cf8" />
            <stop offset="100%" stopColor="#6366f1" />
          </linearGradient>
        </defs>
      </svg>
    </div>
  );
}

// Stats counter component
function StatItem({ number, label, delay }: { number: string; label: string; delay: number }) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), delay);
    return () => clearTimeout(timer);
  }, [delay]);

  return (
    <div className={clsx(styles.statItem, isVisible && styles.visible)}>
      <span className={styles.statNumber}>{number}</span>
      <span className={styles.statLabel}>{label}</span>
    </div>
  );
}

// Feature card component
function FeatureCard({
  icon,
  title,
  description,
  delay,
}: {
  icon: string;
  title: string;
  description: string;
  delay: number;
}) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), delay);
    return () => clearTimeout(timer);
  }, [delay]);

  return (
    <div className={clsx(styles.featureCard, isVisible && styles.visible)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

// Module card component
function ModuleCard({
  number,
  title,
  description,
  chapters,
  link,
  color,
}: {
  number: string;
  title: string;
  description: string;
  chapters: number;
  link: string;
  color: string;
}) {
  return (
    <Link to={link} className={styles.moduleCard}>
      <div className={styles.moduleNumber} style={{ background: color }}>
        {number}
      </div>
      <div className={styles.moduleContent}>
        <h3 className={styles.moduleTitle}>{title}</h3>
        <p className={styles.moduleDescription}>{description}</p>
        <div className={styles.moduleChapters}>
          <span>📚 {chapters} Chapters</span>
          <span className={styles.moduleArrow}>→</span>
        </div>
      </div>
    </Link>
  );
}

// Neural Network / Nervous System Animation Component
function NervousSystemVisual() {
  const [nodes, setNodes] = useState<{ x: number; y: number; id: number }[]>([]);
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

  useEffect(() => {
    // Generate static nodes with more "neural" distribution
    const initialNodes = Array.from({ length: 20 }).map((_, i) => ({
      x: 10 + Math.random() * 80,
      y: 10 + Math.random() * 80,
      id: i,
    }));
    setNodes(initialNodes);

    const handleMouseMove = (e: MouseEvent) => {
      const rect = document.getElementById('neural-container')?.getBoundingClientRect();
      if (rect) {
        const x = ((e.clientX - rect.left) / rect.width) * 100;
        const y = ((e.clientY - rect.top) / rect.height) * 100;
        setMousePos({ x, y });
      }
    };
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <div id="neural-container" className={styles.nervousSystemContainer}>
      <svg className={styles.neuralSvg} viewBox="0 0 100 100" preserveAspectRatio="none">
        <defs>
          <radialGradient id="nodeGlow" cx="50%" cy="50%" r="50%">
            <stop offset="0%" stopColor="#3b82f6" stopOpacity="1" />
            <stop offset="100%" stopColor="#3b82f6" stopOpacity="0" />
          </radialGradient>
        </defs>
        {/* Connection lines */}
        {nodes.map((node, i) =>
          nodes.slice(i + 1).map((other) => {
            const dist = Math.hypot(node.x - other.x, node.y - other.y);
            if (dist < 25) {
              return (
                <line
                  key={`${node.id}-${other.id}`}
                  x1={node.x}
                  y1={node.y}
                  x2={other.x}
                  y2={other.y}
                  className={styles.neuralLine}
                  style={{ opacity: (1 - dist / 25) * 0.5 }}
                />
              );
            }
            return null;
          })
        )}
        {/* Interaction lines to mouse */}
        {nodes.map((node) => {
          const dist = Math.hypot(node.x - mousePos.x, node.y - mousePos.y);
          if (dist < 25) {
            return (
              <line
                key={`mouse-${node.id}`}
                x1={node.x}
                y1={node.y}
                x2={mousePos.x}
                y2={mousePos.y}
                className={styles.mouseLine}
                style={{ opacity: (1 - dist / 25) * 0.8 }}
              />
            );
          }
          return null;
        })}
        {/* Nodes */}
        {nodes.map((node) => (
          <g key={node.id} className={styles.nodeGroup}>
            <circle
              cx={node.x}
              cy={node.y}
              r="1.5"
              fill="url(#nodeGlow)"
              className={styles.neuralNodePulse}
            />
            <circle
              cx={node.x}
              cy={node.y}
              r="0.5"
              fill="#fff"
              className={styles.neuralNodeCore}
            />
          </g>
        ))}
      </svg>
      <div className={styles.neuralGlow} />
    </div>
  );
}

// Interactive Hero Wrapper
function InteractiveHero() {
  const [isActive, setIsActive] = useState(false);
  const [messageIndex, setMessageIndex] = useState(0);

  const messages = [
    "System Initialized. 🧠",
    "Robotic Nervous System Online. ⚡",
    "Frontier 2026 Core Loaded. 🚀",
    "Neural pathways active... Welcome."
  ];

  useEffect(() => {
    const timer = setTimeout(() => {
      setIsActive(true);
      const interval = setInterval(() => {
        setMessageIndex((prev) => (prev + 1) % messages.length);
      }, 3000);
      return () => clearInterval(interval);
    }, 1000);
    return () => clearTimeout(timer);
  }, []);

  return (
    <div className={clsx(styles.interactiveHero, isActive && styles.heroActive)}>
      <NervousSystemVisual />
      <div className={styles.speechBubble}>
        <div className={styles.typingIndicator}>
          <span className="typing-dot" />
          <span className="typing-dot" />
          <span className="typing-dot" />
        </div>
        {messages[messageIndex]}
      </div>
    </div>
  );
}


// Hero section
function HeroSection() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      {/* Animated background */}
      <div className={styles.heroBackground}>
        <div className={styles.gridLines} />
        <div className={styles.glowOrb1} />
        <div className={styles.glowOrb2} />
        <div className={styles.glowOrb3} />
      </div>

      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          {/* Badge */}
          <div className={styles.heroBadge}>
            <span className={styles.badgeDot} />
            Open Source Textbook • Free Forever
          </div>

          {/* Title */}
          <Heading as="h1" className={clsx(styles.heroTitle, styles.neonTitle)}>
            <span className={styles.titleLine}>Physical AI &</span>
            <span className={clsx(styles.titleLine, styles.gradientText)}>
              Humanoid Robotics Book
            </span>
          </Heading>

          {/* Subtitle */}
          <p className={styles.heroSubtitle}>
            <strong>2026 Frontier Specification:</strong> Master ROS 2 Kilted Kaiju, NVIDIA Thor Platforms, and VLA Foundation Models.
            The definitive guide to building intelligent physical robots for the Blackwell era.
          </p>

          {/* CTA Buttons */}
          <div className={styles.heroCta}>
            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/docs/M1/C1/m1-c1-s1"
            >
              Start Reading
              <span className={styles.buttonIcon}>→</span>
            </Link>
            <Link
              className={clsx('button', 'button--lg', styles.secondaryButton)}
              to="/docs/glossary"
            >
              Glossary
            </Link>
          </div>

          {/* Stats */}
          <div className={styles.heroStats}>
            <StatItem number="87" label="Sections" delay={200} />
            <StatItem number="4" label="Modules" delay={400} />
            <StatItem number="2026" label="Edition" delay={600} />
            <StatItem number="∞" label="Knowledge" delay={800} />
          </div>
        </div>

        {/* Interactive Neural Visual */}
        <div className={styles.heroVisual}>
          <InteractiveHero />
        </div>
      </div>

      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollLine} />
      </div>
    </header>
  );
}

// Features section
function FeaturesSection() {
  const features = [
    {
      icon: '🏎️',
      title: 'RTX 5080/6080',
      description: 'Optimized for Blackwell architecture and massive VLA model inference.',
    },
    {
      icon: '🧠',
      title: 'Jetson Thor',
      description: 'Native support for the 2025 humanoid SoC for real-time edge intelligence.',
    },
    {
      icon: '🦖',
      title: 'Kilted Kaiju',
      description: 'Master the 2025 ROS 2 LTS with optimized rmw for high-bandwidth humanoids.',
    },
    {
      icon: '🦾',
      title: 'Unitree G1',
      description: 'Full humanoid integration using the latest 2025 SDK and Sim-to-Real paths.',
    },
    {
      icon: '👁️',
      title: 'YOLOv11',
      description: 'State-of-the-art perception suite with hardware-accelerated VPI 3.x.',
    },
    {
      icon: '🌐',
      title: 'Gazebo Ionic',
      description: 'High-fidelity physics simulation using the latest 9.x Ionic engine.',
    },
  ];

  return (
    <section id="github-options" className={`${styles.featuresSection} ${styles.sectionAnimate}`}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Features</span>
          <h2 className={styles.sectionTitle}>
            Everything you need to build
            <span className={styles.gradientText}> intelligent robots</span>
          </h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive curriculum designed for all skill levels,
            from curious beginners to seasoned robotics engineers.
          </p>
        </div>

        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <FeatureCard
              key={index}
              icon={feature.icon}
              title={feature.title}
              description={feature.description}
              delay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

// Modules section
function ModulesSection() {
  const modules = [
    {
      number: 'M1',
      title: 'The Robotic Nervous System',
      description: 'ROS 2 Kilted Kaiju foundations, custom rmw optimization, and hardware abstraction.',
      chapters: 3,
      link: '/docs/M1/C1/m1-c1-s1',
      color: 'linear-gradient(135deg, #6366f1, #8b5cf6)',
    },
    {
      number: 'M2',
      title: 'The Digital Twin Hallucination',
      description: 'Gazebo Ionic physics, Unity HRI, and extreme sim-to-real transfer protocols.',
      chapters: 3,
      link: '/docs/M2/C1/m2-c1-s1',
      color: 'linear-gradient(135deg, #8b5cf6, #a855f7)',
    },
    {
      number: 'M3',
      title: 'The AI-Robot Awakening',
      description: 'NVIDIA Isaac Sim 2025, Blackwell-accelerated perception, and RL gait training.',
      chapters: 3,
      link: '/docs/M3/C1/m3-c1-s1',
      color: 'linear-gradient(135deg, #a855f7, #d946ef)',
    },
    {
      number: 'M4',
      title: 'The Vision-Language-Action Embodiment',
      description: 'Hierarchical VLA policies, VILA-8B integration, and the Autonomous Humanoid Capstone.',
      chapters: 3,
      link: '/docs/M4/C1/m4-c1-s1',
      color: 'linear-gradient(135deg, #d946ef, #ec4899)',
    },
  ];

  return (
    <section className={`${styles.modulesSection} ${styles.sectionAnimate}`}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Curriculum</span>
          <h2 className={styles.sectionTitle}>
            Four modules.
            <span className={styles.gradientText}> Complete mastery.</span>
          </h2>
          <p className={styles.sectionSubtitle}>
            Progress through structured learning paths that take you from
            beginner concepts to production-ready implementations.
          </p>
        </div>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <ModuleCard key={index} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

// AI Features section
function AIFeaturesSection() {
  return (
    <section id="free-account" className={`${styles.aiSection} ${styles.sectionAnimate}`}>
      <div className={styles.sectionContainer}>
        <div className={styles.aiContent}>
          <div className={styles.aiText}>
            <span className={styles.sectionBadge}>AI-Powered</span>
            <h2 className={styles.sectionTitle}>
              Learn smarter with
              <span className={styles.gradientText}> AI assistance</span>
            </h2>
            <p className={styles.sectionSubtitle}>
              Our intelligent features adapt to your learning style and help you
              master robotics faster than ever before.
            </p>

            <div className={styles.aiFeaturesList}>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>💬</div>
                <div>
                  <h4>RAG Chatbot</h4>
                  <p>Ask questions and get instant answers from the textbook content.</p>
                </div>
              </div>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>✨</div>
                <div>
                  <h4>Personalization</h4>
                  <p>Content adapts to your background, goals, and preferred depth.</p>
                </div>
              </div>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>🌐</div>
                <div>
                  <h4>Urdu Translation</h4>
                  <p>Access content in Urdu for better understanding.</p>
                </div>
              </div>
            </div>

            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/auth/signup"
            >
              Create Free Account
            </Link>
          </div>

          <div className={styles.aiVisual}>
            <div className={styles.chatPreview}>
              <div className={styles.chatHeader}>
                <span className={styles.chatDot} />
                <span className={styles.chatDot} />
                <span className={styles.chatDot} />
                <span>AI Assistant</span>
              </div>
              <div className={styles.chatMessages}>
                <div className={styles.chatUser}>
                  How do I create a ROS 2 node?
                </div>
                <div className={styles.chatBot}>
                  <p>To create a ROS 2 node, follow these steps:</p>
                  <code>ros2 pkg create my_package</code>
                  <p>Then implement your node class extending Node...</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// CTA section
function CTASection() {
  return (
    <section className={`${styles.ctaSection} ${styles.sectionAnimate}`}>
      <div className={styles.sectionContainer}>
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>
            Ready to build the
            <span className={styles.gradientText}> future of robotics?</span>
          </h2>
          <p className={styles.ctaSubtitle}>
            Join thousands of engineers, researchers, and enthusiasts learning
            Physical AI and Humanoid Robotics.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/docs/M1/C1/m1-c1-s1"
            >
              Start Learning Now
              <span className={styles.buttonIcon}>→</span>
            </Link>
            <Link
              className={clsx('button', 'button--lg', styles.secondaryButton)}
              to="https://github.com/junaid/Hackathon-1"
            >
              ⭐ Star on GitHub
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Robot showcase item
function RobotShowcaseItem({ name, src, delay }: { name: string; src: string; delay: number }) {
  return (
    <div className={styles.robotShowcaseItem} style={{ animationDelay: `${delay}ms` }}>
      <div className={styles.videoWrapper}>
        <iframe
          src={src}
          title={name}
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
          allowFullScreen
        ></iframe>
      </div>
      <div className={styles.robotInfo}>
        <span className={styles.robotName}>{name}</span>
        <span className={styles.robotStatus}>Status: Operational</span>
      </div>
    </div>
  );
}


// YouTube Video Showcase Item
function YouTubeVideoItem({ title, videoId, delay }: { title: string; videoId: string; delay: number }) {
  return (
    <div className={styles.youtubeVideoItem} style={{ animationDelay: `${delay}ms` }}>
      <div className={styles.videoWrapper}>
        <iframe
          src={`https://www.youtube.com/embed/${videoId}`}
          title={title}
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
          allowFullScreen
        ></iframe>
      </div>
      <div className={styles.videoInfo}>
        <span className={styles.videoTitle}>{title}</span>
      </div>
    </div>
  );
}

// Enhanced Module Card Component
function EnhancedModuleCard({
  number,
  title,
  description,
  link,
  color,
  icon
}: {
  number: string;
  title: string;
  description: string;
  link: string;
  color: string;
  icon: string;
}) {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <Link
      to={link}
      className={styles.enhancedModuleCard}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      style={{
        background: color,
        transform: isHovered ? 'translateY(-10px) scale(1.02)' : 'translateY(0) scale(1)',
        transition: 'all 0.3s ease'
      }}
    >
      <div className={styles.enhancedModuleContent}>
        <div className={styles.enhancedModuleIcon}>{icon}</div>
        <div className={styles.enhancedModuleText}>
          <h3 className={styles.enhancedModuleTitle}>{title}</h3>
          <p className={styles.enhancedModuleDescription}>{description}</p>
        </div>
        <div className={styles.enhancedModuleNumber}>{number}</div>
      </div>
    </Link>
  );
}

// VS Code Animated Demo Component
function VSCodeDemo() {
  const [activeTab, setActiveTab] = useState('code');
  const [terminalOpen, setTerminalOpen] = useState(false);

  useEffect(() => {
    const interval = setInterval(() => {
      setActiveTab(prev => prev === 'code' ? 'agent' : prev === 'agent' ? 'terminal' : 'code');
      if (!terminalOpen && activeTab === 'terminal') {
        setTerminalOpen(true);
      }
    }, 3000);

    return () => clearInterval(interval);
  }, [terminalOpen, activeTab]);

  return (
    <div className={styles.vscodeDemo}>
      <div className={styles.vscodeWindow}>
        {/* VS Code Title Bar */}
        <div className={styles.vscodeTitleBar}>
          <div className={styles.vscodeTrafficLights}>
            <div className={styles.trafficLight}></div>
            <div className={styles.trafficLight}></div>
            <div className={styles.trafficLight}></div>
          </div>
          <div className={styles.vscodeTitle}>physical-ai-book/main.py - Visual Studio Code</div>
        </div>

        {/* VS Code Tabs */}
        <div className={styles.vscodeTabs}>
          <div className={`${styles.vscodeTab} ${activeTab === 'code' ? styles.activeTab : ''}`}>
            main.py
          </div>
          <div className={`${styles.vscodeTab} ${activeTab === 'agent' ? styles.activeTab : ''}`}>
            agent.tsx
          </div>
        </div>

        {/* VS Code Editor */}
        <div className={styles.vscodeEditor}>
          <pre className={styles.codeSnippet}>
            <code>
{`# Physical AI & Humanoid Robotics
import rospy
from geometry_msgs.msg import Twist

class HumanoidController:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('humanoid_controller')

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.0
        self.pub.publish(twist)

if __name__ == '__main__':
    controller = HumanoidController()
    controller.move_forward()`}
            </code>
          </pre>
        </div>

        {/* VS Code Terminal */}
        <div className={`${styles.vscodeTerminal} ${terminalOpen ? styles.terminalOpen : ''}`}>
          <div className={styles.terminalHeader}>
            <span>bash</span>
          </div>
          <div className={styles.terminalContent}>
            <div className={styles.terminalLine}>
              <span className={styles.terminalPrompt}>$</span>
              <span className={styles.terminalCommand}>npm run build</span>
            </div>
            <div className={styles.terminalOutput}>
              Compiled successfully!
              <br />
              ./src/App.js → ./dist/App.js
            </div>
            <div className={styles.terminalLine}>
              <span className={styles.terminalPrompt}>$</span>
              <span className={styles.terminalCommand}>roslaunch humanoid_bringup bringup.launch</span>
            </div>
            <div className={styles.terminalOutput}>
              [INFO] [1234567890.123]: Humanoid robot initialized
              <br />
              [INFO] [1234567890.456]: Controllers loaded successfully
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

// Hero Video Section (FIXED - MOVED TO TOP)
function HeroVideoSection() {
  return (
    <section className={styles.heroVideoSection}>
      <div className={styles.heroVideoContainer}>
        <div className={styles.videoWrapper}>
          <iframe
            src="https://www.youtube.com/embed/nzflxCHT4vw?autoplay=1&mute=1&loop=1&playlist=nzflxCHT4vw&controls=0&modestbranding=1"
            title="UBTECH Walker S2 - World's First Mass Delivery of Humanoid Robot"
            frameBorder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowFullScreen
          ></iframe>
        </div>
      </div>
    </section>
  );
}

// Additional YouTube Videos Section
function AdditionalVideosSection() {
  return (
    <section className={styles.additionalVideosSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Featured Robotics Content</span>
          <h2 className={styles.sectionTitle}>
            Watch <span className={styles.gradientText}>Physical AI</span> in Action
          </h2>
          <p className={styles.sectionSubtitle}>
            See the latest advancements in humanoid robotics and AI
          </p>
        </div>

        <div className={styles.additionalVideosGrid}>
          <div className={styles.videoCard}>
            <div className={styles.videoWrapperSmall}>
              <iframe
                src="https://www.youtube.com/embed/HOoRnv3lA0k"
                title="Scaling Helix - Laundry"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
              ></iframe>
            </div>
            <h3>Scaling Helix - Laundry</h3>
          </div>

          <div className={styles.videoCard}>
            <div className={styles.videoWrapperSmall}>
              <iframe
                src="https://www.youtube.com/embed/Eu5mYMavctM"
                title="Introducing Figure 03"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
              ></iframe>
            </div>
            <h3>Introducing Figure 03</h3>
          </div>

          <div className={styles.videoCard}>
            <div className={styles.videoWrapperSmall}>
              <iframe
                src="https://www.youtube.com/embed/n1Mi3ISXNjc"
                title="Engine AI T800 vs Tesla Optimus V3 vs Figure 03 (AI NEWS)"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
              ></iframe>
            </div>
            <h3>AI News: T800 vs Optimus vs Figure 03</h3>
          </div>
        </div>
      </div>
    </section>
  );
}

// Enhanced Content Section
function EnhancedContentSection() {
  return (
    <section className={styles.enhancedContentSection}>
      <div className={styles.sectionContainer}>
        {/* Part 1: YouTube Videos */}
        <div className={styles.youtubeSection}>
          <div className={styles.sectionHeader}>
            <span className={styles.sectionBadge}>Featured Videos</span>
            <h2 className={styles.sectionTitle}>
              Watch <span className={styles.gradientText}>Physical AI</span> in Action
            </h2>
            <p className={styles.sectionSubtitle}>
              See the latest advancements in humanoid robotics and AI
            </p>
          </div>
          <div className={styles.youtubeGrid}>
            <YouTubeVideoItem title="Figure 02 Humanoid Robot" videoId="Q5XNoIK8nbM" delay={100} />
            <YouTubeVideoItem title="Unitree G1 Walking Demo" videoId="Me8_vofae98" delay={300} />
            <YouTubeVideoItem title="Tesla Optimus Gen 2" videoId="cpraXaw7dyc" delay={500} />
            <YouTubeVideoItem title="NVIDIA Blackwell Architecture" videoId="JNVYEIzY6tg" delay={700} />
          </div>
        </div>

        {/* Part 2: Enhanced Modules */}
        <div className={styles.enhancedModulesSection}>
          <div className={styles.sectionHeader}>
            <span className={styles.sectionBadge}>Complete Curriculum</span>
            <h2 className={styles.sectionTitle}>
              Explore All <span className={styles.gradientText}>Learning Modules</span>
            </h2>
            <p className={styles.sectionSubtitle}>
              From fundamentals to advanced topics
            </p>
          </div>
          <div className={styles.enhancedModulesGrid}>
            <EnhancedModuleCard
              number="M1"
              title="The Robotic Nervous System"
              description="ROS 2 Kilted Kaiju foundations, custom rmw optimization, and hardware abstraction."
              link="/docs/M1/C1/m1-c1-s1"
              color="linear-gradient(135deg, #6366f1, #8b5cf6)"
              icon="🧠"
            />
            <EnhancedModuleCard
              number="M2"
              title="The Digital Twin Hallucination"
              description="Gazebo Ionic physics, Unity HRI, and extreme sim-to-real transfer protocols."
              link="/docs/M2/C1/m2-c1-s1"
              color="linear-gradient(135deg, #8b5cf6, #a855f7)"
              icon="🔄"
            />
            <EnhancedModuleCard
              number="M3"
              title="The AI-Robot Awakening"
              description="NVIDIA Isaac Sim 2025, Blackwell-accelerated perception, and RL gait training."
              link="/docs/M3/C1/m3-c1-s1"
              color="linear-gradient(135deg, #a855f7, #d946ef)"
              icon="👁️"
            />
            <EnhancedModuleCard
              number="M4"
              title="The Vision-Language-Action Embodiment"
              description="Hierarchical VLA policies, VILA-8B integration, and the Autonomous Humanoid Capstone."
              link="/docs/M4/C1/m4-c1-s1"
              color="linear-gradient(135deg, #d946ef, #ec4899)"
              icon="🤖"
            />
            <EnhancedModuleCard
              number="G"
              title="Glossary"
              description="Comprehensive terminology and definitions for robotics and AI concepts."
              link="/docs/glossary"
              color="linear-gradient(135deg, #ec4899, #f43f5e)"
              icon="📖"
            />
            <EnhancedModuleCard
              number="A"
              title="Self Assessment"
              description="Evaluate your knowledge and track your progress through interactive quizzes."
              link="/docs/self-assessment"
              color="linear-gradient(135deg, #f43f5e, #f97316)"
              icon="🎯"
            />
          </div>
        </div>

        {/* Part 3: VS Code Demo */}
        <div className={styles.vscodeDemoSection}>
          <div className={styles.sectionHeader}>
            <span className={styles.sectionBadge}>Live Demo</span>
            <h2 className={styles.sectionTitle}>
              Experience <span className={styles.gradientText}>Development</span> in Action
            </h2>
            <p className={styles.sectionSubtitle}>
              Watch how our code comes to life in a real development environment
            </p>
          </div>
          <VSCodeDemo />
        </div>
      </div>
    </section>
  );
}

// Robot showcase section
function RobotShowcase() {
  return (
    <>
      <HeroVideoSection />
      <AdditionalVideosSection />
    </>
  );
}

// Main Home component
export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(true);

  // Scroll-triggered animations
  useEffect(() => {
    const handleScroll = () => {
      // Add scroll-triggered animations here
      const scrollPosition = window.scrollY;
      const sections = document.querySelectorAll('.section-animate');

      sections.forEach((section, index) => {
        const sectionTop = section.getBoundingClientRect().top;
        const windowHeight = window.innerHeight;

        if (sectionTop < windowHeight * 0.75) {
          section.classList.add('section-visible');
        }
      });
    };

    window.addEventListener('scroll', handleScroll);
    handleScroll(); // Initial check

    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <Layout
      title="Physical AI & Humanoid Robotics Book"
      description="The definitive 2026 guide to building intelligent physical robots. Master ROS 2 Kilted Kaiju, Simulation, and VLA policies."
    >
      <HeroSection />
      <RobotShowcase />
      <VscodeInterface />
      <EnhancedContentSection />
      <FeaturesSection />
      <AIFeaturesSection />
      <CTASection />

      {/* Floating Quick Start Button */}
      <button
        className={styles.quickStartButton}
        onClick={() => {
          document.querySelector('#github-options')?.scrollIntoView({ behavior: 'smooth' });
        }}
        aria-label="Quick Start"
      >
        🚀
      </button>
    </Layout>
  );
}
