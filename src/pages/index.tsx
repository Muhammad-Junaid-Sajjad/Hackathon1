import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import VscodeInterface from '@site/src/components/VscodeInterface';
import RobotViewer from '@site/src/components/RobotViewer';

// Simple Stat Item Component
function StatItem({ number, label }: { number: string; label: string }) {
  return (
    <div className={styles.statItem}>
      <span className={styles.statNumber}>{number}</span>
      <span className={styles.statLabel}>{label}</span>
    </div>
  );
}

// Hero Section
function Hero() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gridLines} />
      </div>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>2026 Frontier Edition</div>
          <Heading as="h1" className={clsx(styles.heroTitle, styles.gradientText)}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>
            {siteConfig.tagline}
          </p>
          <div className={styles.heroCta}>
            <Link className={clsx('button button--lg button--primary', styles.primaryButton)} to="/docs/M1/C1/m1-c1-s1">
              Start Learning →
            </Link>
            <Link className={clsx('button button--lg button--secondary', styles.secondaryButton)} to="/docs/intro">
              Explore Curriculum
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

// Video Section with UBTECH video and 4-grid
function VideoSection() {
  return (
    <section className={styles.videoSection}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-horiz--md">🤖 Robotics in Action</h2>

        {/* UBTECH Walker S2 Video - Full Width */}
        <div className={styles.ubtechVideoContainer}>
          <div className={styles.videoWrapperWide}>
            <iframe
              src="https://www.youtube.com/embed/nzflxCHT4vw"
              title="UBTECH Walker S2"
              frameBorder="0"
              allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
              allowFullScreen
              className={styles.videoFrame}
            ></iframe>
          </div>
        </div>

        {/* Grid of 4 videos */}
        <div className={styles.videoGrid}>
          <div className={styles.videoItem}>
            <div className={styles.videoWrapper}>
              <iframe
                src="https://www.youtube.com/embed/Eu5mYMavctM"
                title="Figure 03"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
                className={styles.videoFrame}
              ></iframe>
            </div>
            <h3 className="text--center padding-horiz--sm">Figure 03</h3>
          </div>

          <div className={styles.videoItem}>
            <div className={styles.videoWrapper}>
              <iframe
                src="https://www.youtube.com/embed/n1Mi3ISXNjc"
                title="Humanoid AI News"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
                className={styles.videoFrame}
              ></iframe>
            </div>
            <h3 className="text--center padding-horiz--sm">Humanoid AI News</h3>
          </div>

          <div className={styles.videoItem}>
            <div className={styles.videoWrapper}>
              <iframe
                src="https://www.youtube.com/embed/nzflxCHT4vw"
                title="UBTECH Walker S2"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
                className={styles.videoFrame}
              ></iframe>
            </div>
            <h3 className="text--center padding-horiz--sm">UBTECH Walker S2</h3>
          </div>

          <div className={styles.videoItem}>
            <div className={styles.videoWrapper}>
              <iframe
                src="https://www.youtube.com/embed/cpraXaw7dyc"
                title="Tesla Optimus Gen 2"
                frameBorder="0"
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                allowFullScreen
                className={styles.videoFrame}
              ></iframe>
            </div>
            <h3 className="text--center padding-horiz--sm">Tesla Optimus Gen 2</h3>
          </div>
        </div>
      </div>
    </section>
  );
}

// Modules Grid
function ModulesGrid() {
  const modules = [
    { n: 'M1', t: 'Nervous System', d: 'ROS 2 Foundations', c: '#6366f1' },
    { n: 'M2', t: 'Digital Twin', d: 'Physics & Gazebo', c: '#8b5cf6' },
    { n: 'M3', t: 'AI Awakening', d: 'Isaac Sim & RL', c: '#a855f7' },
    { n: 'M4', t: 'Embodiment', d: 'Vision-Language-Action', c: '#d946ef' },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className="text--center padding-horiz--md">Complete Curriculum</h2>
        <div className={styles.modulesGrid}>
          {modules.map((m, i) => (
            <Link to={`/docs/M${i+1}/C1/m${i+1}-c1-s1`} key={i} className={styles.moduleCard}>
              <div className={styles.moduleNumber} style={{ background: m.c }}>{m.n}</div>
              <div className={styles.moduleContent}>
                <h3 className={styles.moduleTitle}>{m.t}</h3>
                <p className={styles.moduleDescription}>{m.d}</p>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

// Stats Section
function StatsDashboard() {
  return (
    <section className={styles.statsSection}>
      <div className="container padding-vert--lg">
        <div className={styles.statsGrid}>
          <StatItem number="10,000+" label="Students Learning" />
          <StatItem number="87+" label="Sections Completed" />
          <StatItem number="1,500+" label="Projects Built" />
          <StatItem number="45+" label="Countries" />
        </div>
      </div>
    </section>
  );
}

// Technology Logos Showcase
function TechLogosShowcase() {
  const techLogos = [
    { name: 'ROS 2', logo: 'https://cdn.jsdelivr.net/gh/devicons/devicon/icons/ros/ros-plain.svg' },
    { name: 'NVIDIA', logo: 'https://cdn.jsdelivr.net/gh/devicons/devicon/icons/nvidia/nvidia-original.svg' },
    { name: 'Gazebo', logo: 'https://gazebosim.org/assets/images/logos/gazebo.svg' },
    { name: 'PyTorch', logo: 'https://cdn.jsdelivr.net/gh/devicons/devicon/icons/pytorch/pytorch-original.svg' },
    { name: 'TensorFlow', logo: 'https://cdn.jsdelivr.net/gh/devicons/devicon/icons/tensorflow/tensorflow-original.svg' },
    { name: 'YOLO', logo: 'https://upload.wikimedia.org/wikipedia/commons/5/5f/YOLO.svg' },
    { name: 'OpenCV', logo: 'https://cdn.jsdelivr.net/gh/devicons/devicon/icons/opencv/opencv-original.svg' },
  ];

  return (
    <section className={styles.techShowcase}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-horiz--md">Technologies You'll Master</h2>
        <div className={styles.techGrid}>
          {techLogos.map((tech, i) => (
            <div key={i} className={styles.techItem}>
              <img src={tech.logo} alt={tech.name} className={styles.techLogo} />
              <span className={styles.techName}>{tech.name}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// VS Code Demo Section (Simplified)
function VSCodeDemo() {
  return (
    <section className={styles.vscodeSection}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-horiz--md">💻 Live Development Environment</h2>
        <p className="text--center padding-horiz--md">Experience hands-on learning with our integrated VS Code simulator. Write, test, and debug robotics code in real-time.</p>
        <div className={styles.vscodeContainer}>
          <VscodeInterface />
        </div>
      </div>
    </section>
  );
}

// Quick Feature Highlights
function FeatureHighlights() {
  const features = [
    { icon: '🤖', title: 'Build Real Robots', desc: 'From simulation to hardware implementation' },
    { icon: '💡', title: 'AI-Powered Learning', desc: 'Instant assistance with RAG-based Q&A' },
    { icon: '🌍', title: 'Global Community', desc: 'Connect with learners worldwide' },
  ];

  return (
    <section className={styles.featureHighlights}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-horiz--md">Why Choose Our Platform</h2>
        <div className={styles.featureGrid}>
          {features.map((feature, i) => (
            <div key={i} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDesc}>{feature.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robotics Textbook - 2026 Edition">
      <Hero />

      <main>
        <StatsDashboard />

        <section className="container padding-vert--xl">
          <div className="row">
            <div className="col col--6">
              <h2>🤖 Interactive 3D Robot Viewer</h2>
              <p>Explore the humanoid robot anatomy with our interactive 3D viewer. Understand the nervous system architecture and component relationships.</p>
            </div>
            <div className="col col--6">
              <RobotViewer />
            </div>
          </div>
        </section>

        <VideoSection />

        <ModulesGrid />

        <TechLogosShowcase />

        <VSCodeDemo />

        <FeatureHighlights />

        <section className="padding-vert--xl text--center" style={{ background: 'var(--ifm-color-primary-dark)' }}>
          <div className="container">
            <h2 className={styles.gradientText}>Ready to Build the Future?</h2>
            <p style={{ color: 'rgba(255,255,255,0.8)', maxWidth: '600px', margin: '0 auto 2rem' }}>
              Join thousands of students mastering the next generation of robotics and AI.
            </p>
            <Link className="button button--lg button--secondary" to="/docs/M1/C1/m1-c1-s1">
              Start Learning Today
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
