/**
 * Homepage - Physical AI & Humanoid Robotics Textbook
 *
 * Design inspired by: Figure.ai (futuristic robotics), GitHub (clean), Panaversity (educational)
 */

import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

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

// Figure-style Robotic Face Component
function RoboticFace() {
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      // Calculate normalized mouse position (-1 to 1)
      const x = (e.clientX / window.innerWidth) * 2 - 1;
      const y = (e.clientY / window.innerHeight) * 2 - 1;
      setMousePos({ x, y });
    };
    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <div className={styles.roboticFaceContainer} style={{
      transform: `perspective(1000px) rotateY(${mousePos.x * 10}deg) rotateX(${mousePos.y * -10}deg)`
    }}>
      <div className={styles.roboticFace}>
        <div className={styles.faceGlow} />
        <div className={styles.eyeContainer}>
          <div className={styles.roboticEye}>
            <div className={styles.eyeIris} />
            <div className={styles.eyePupil} />
          </div>
          <div className={styles.roboticEye}>
            <div className={styles.eyeIris} />
            <div className={styles.eyePupil} />
          </div>
        </div>
        <div className={styles.breathingCore} />
      </div>
    </div>
  );
}

// Interactive Book Component (Simplified for sequence)
function InteractiveBook() {
  const [isActive, setIsActive] = useState(false);
  const [messageIndex, setMessageIndex] = useState(0);

  const messages = [
    "Hello! I am your AI Guide. 👋",
    "Ready to explore the Future of Robotics?",
    "Okay! Let's dive into the Book! 🚀",
    "Closing data stream... see you inside!"
  ];

  // Auto-greeting effect
  useEffect(() => {
    const autoGreetingTimer = setTimeout(() => {
      if (!isActive) {
        setIsActive(true);
        setMessageIndex(0);
        setTimeout(() => setIsActive(false), 3000);
      }
    }, 2000);

    return () => clearTimeout(autoGreetingTimer);
  }, []);

  const toggleBook = (e?: React.MouseEvent) => {
    if (e) e.preventDefault();
    if (!isActive) {
      setIsActive(true);
      setMessageIndex(0);
      setTimeout(() => setMessageIndex(1), 2000);
      setTimeout(() => setMessageIndex(2), 4000);
      setTimeout(() => setMessageIndex(3), 6000);
      setTimeout(() => setIsActive(false), 8000);
    } else {
      setIsActive(false);
    }
  };

  return (
    <div
      className={clsx(styles.interactiveHero, isActive && styles.bookActive)}
      onClick={toggleBook}
    >
      <div className={styles.heroVisualContent}>
        <RoboticFace />

        <div className={styles.agentAvatar}>
          <div className={styles.robotFace}>✨🤖</div>
        </div>

        <div className={styles.speechBubble}>
          {messages[messageIndex]}
        </div>
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
            <strong>2025 Frontier Specification:</strong> Master ROS 2 Kilted Kaiju, NVIDIA Thor Platforms, and VLA Foundation Models.
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
            <StatItem number="2025" label="Edition" delay={600} />
            <StatItem number="∞" label="Knowledge" delay={800} />
          </div>
        </div>

        {/* Interactive Book Illustration */}
        <div className={styles.heroVisual}>
          <InteractiveBook />
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
    <section id="github-options" className={styles.featuresSection}>
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
    <section className={styles.modulesSection}>
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
    <section id="free-account" className={styles.aiSection}>
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
    <section className={styles.ctaSection}>
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

// Robot Showcase Section
function RobotShowcase() {
  return (
    <section className={styles.showcaseSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Frontier Hardware</span>
          <h2 className={styles.sectionTitle}>
            Built for the
            <span className={styles.gradientText}> 2025 Generation</span>
          </h2>
          <p className={styles.sectionSubtitle}>
            Witness the physical manifestation of the digital brain. Our curriculum integrates with the leading humanoid platforms on earth.
          </p>
        </div>
        <div id="video-showcase" className={styles.robotGrid}>
          <RobotShowcaseItem name="Figure 02" src="https://www.youtube.com/embed/Q5XNoIK8nbM" delay={100} />
          <RobotShowcaseItem name="Unitree G1" src="https://www.youtube.com/embed/Me8_vofae98" delay={300} />
          <RobotShowcaseItem name="Tesla Optimus Gen 2" src="https://www.youtube.com/embed/cpraXaw7dyc" delay={500} />
        </div>
      </div>
    </section>
  );
}

// Main Home component
export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Physical AI & Humanoid Robotics Book"
      description="The definitive 2025 guide to building intelligent physical robots. Master ROS 2 Kilted Kaiju, Simulation, and VLA policies."
    >
      <HeroSection />
      <RobotShowcase />
      <FeaturesSection />
      <ModulesSection />
      <AIFeaturesSection />
      <CTASection />
    </Layout>
  );
}
