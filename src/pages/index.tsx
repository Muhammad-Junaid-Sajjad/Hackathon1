import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import VscodeInterface from '@site/src/components/VscodeInterface';
import RobotViewer from '@site/src/components/RobotViewer';

// --- Sub-components ---

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

function YouTubeVideoGrid() {
  const videos = [
    { id: 'HOoRnv3lA0k', title: 'Scaling Helix - Laundry' },
    { id: 'Eu5mYMavctM', title: 'Introducing Figure 03' },
    { id: 'n1Mi3ISXNjc', title: 'Humanoid AI News' },
    { id: 'nzflxCHT4vw', title: 'UBTECH Walker S2' },
  ];
  return (
    <section className={styles.additionalVideosSection}>
      <div className="container">
        <h2 className="text-center mb-12">🛠️ Robotics in Action</h2>
        <div className={styles.additionalVideosGrid}>
          {videos.map((v, i) => (
            <div key={i} className={styles.videoCard}>
              <div className={styles.videoWrapperSmall}>
                <iframe src={`https://www.youtube.com/embed/${v.id}`} title={v.title} frameBorder="0" allowFullScreen></iframe>
              </div>
              <h3 className="text-center mt-4" style={{ fontSize: '0.9rem' }}>{v.title}</h3>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningPathTimeline() {
  const milestones = [
    { title: 'Beginner', modules: 'Module 1', skills: 'ROS 2 Foundations' },
    { title: 'Intermediate', modules: 'Module 2', skills: 'Physics & Simulation' },
    { title: 'Advanced', modules: 'Module 3', skills: 'NVIDIA Isaac & RL' },
    { title: 'Expert', modules: 'Module 4', skills: 'Embodied VLA' }
  ];
  return (
    <section className={styles.timelineSection} style={{ padding: '6rem 0', background: 'var(--color-bg-secondary)' }}>
      <div className="container">
        <h2 className="text-center mb-12">🎓 Learning Path Timeline</h2>
        <div style={{ display: 'flex', justifyContent: 'space-between', flexWrap: 'wrap', gap: '2rem' }}>
          {milestones.map((ms, i) => (
            <div key={i} style={{ flex: 1, minWidth: '200px', padding: '2rem', background: 'var(--color-bg-elevated)', borderRadius: '16px', border: '1px solid var(--color-border-light)' }}>
              <div style={{ color: 'var(--ifm-color-primary)', fontWeight: 'bold', fontSize: '1.5rem', marginBottom: '1rem' }}>0{i+1}</div>
              <h3 style={{ margin: 0 }}>{ms.title}</h3>
              <p style={{ color: 'var(--color-text-secondary)', fontSize: '0.9rem' }}>{ms.modules}</p>
              <div style={{ fontSize: '0.8rem', color: 'var(--ifm-color-primary)' }}>{ms.skills}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function RobotComparisonTable() {
  const data = [
    { name: 'Unitree G1', dof: '43', height: '1.27m', battery: '2h', price: '$16k' },
    { name: 'Figure 03', dof: '50+', height: '1.6m', battery: '5h', price: 'Enterprise' },
    { name: 'Tesla Optimus', dof: '40', height: '1.73m', battery: '8h', price: '$20k' },
    { name: 'BD Atlas', dof: '28', height: '1.5m', battery: '1h', price: 'R&D' },
  ];
  return (
    <section style={{ padding: '6rem 0' }}>
      <div className="container">
        <h2 className="text-center mb-12">⚙️ Hardware Comparison</h2>
        <div style={{ overflowX: 'auto' }}>
          <table style={{ width: '100%', borderCollapse: 'collapse', background: 'var(--color-bg-elevated)', borderRadius: '16px', overflow: 'hidden' }}>
            <thead>
              <tr style={{ background: 'rgba(59, 130, 246, 0.1)', textAlign: 'left' }}>
                <th style={{ padding: '1.5rem' }}>Robot</th>
                <th style={{ padding: '1.5rem' }}>DOF</th>
                <th style={{ padding: '1.5rem' }}>Height</th>
                <th style={{ padding: '1.5rem' }}>Battery</th>
                <th style={{ padding: '1.5rem' }}>Price</th>
              </tr>
            </thead>
            <tbody>
              {data.map((r, i) => (
                <tr key={i} style={{ borderTop: '1px solid var(--color-border-light)' }}>
                  <td style={{ padding: '1.5rem' }}><strong>{r.name}</strong></td>
                  <td style={{ padding: '1.5rem' }}>{r.dof}</td>
                  <td style={{ padding: '1.5rem' }}>{r.height}</td>
                  <td style={{ padding: '1.5rem' }}>{r.battery}</td>
                  <td style={{ padding: '1.5rem' }}>{r.price}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </section>
  );
}

function TechStackShowcase() {
  const techs = ['ROS 2', 'NVIDIA Isaac', 'Gazebo', 'PyTorch', 'TensorFlow', 'YOLOv11', 'OpenCV', 'Docker'];
  return (
    <section style={{ padding: '4rem 0', background: 'var(--color-bg-primary)' }}>
      <div className="container">
        <h2 className="text-center mb-8">🚀 Technology Stack</h2>
        <div style={{ display: 'flex', flexWrap: 'wrap', justifyContent: 'center', gap: '1rem' }}>
          {techs.map((t, i) => (
            <div key={i} style={{ padding: '0.75rem 1.5rem', background: 'var(--gradient-card)', borderRadius: '99px', border: '1px solid var(--color-border-light)', fontWeight: 'bold', color: 'var(--ifm-color-primary)' }}>{t}</div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsDashboard() {
  return (
    <section style={{ padding: '4rem 0', background: 'var(--color-bg-secondary)' }}>
      <div className="container">
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))', gap: '2rem' }}>
          <StatItem number="10,000+" label="Students Learning" delay={200} />
          <StatItem number="87" label="Sections Completed" delay={400} />
          <StatItem number="1,500+" label="Projects Built" delay={600} />
          <StatItem number="45+" label="Countries" delay={800} />
        </div>
      </div>
    </section>
  );
}

function NewsletterSignup() {
  return (
    <section style={{ padding: '6rem 0', textAlign: 'center', background: 'var(--gradient-hero)' }}>
      <div className="container">
        <h2>📧 Join the Community</h2>
        <p style={{ color: 'rgba(255,255,255,0.8)' }}>Join 5,000+ enthusiasts. Get weekly 2026 robotics updates.</p>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginTop: '2rem' }}>
          <input type="email" placeholder="Enter your email" style={{ padding: '1rem', borderRadius: '8px', border: 'none', width: '300px' }} />
          <button style={{ padding: '1rem 2rem', borderRadius: '8px', border: 'none', background: 'var(--ifm-color-primary)', color: 'white', fontWeight: 'bold', cursor: 'pointer' }}>Subscribe</button>
        </div>
      </div>
    </section>
  );
}

function FAQSection() {
  const faqs = [
    { q: "Suitable for beginners?", a: "Yes, we cover Linux and ROS 2 basics." },
    { q: "Hardware requirements?", a: "NVIDIA GPU recommended for AI simulations." },
    { q: "Course duration?", a: "Typically 3-4 months of focused study." }
  ];
  return (
    <section style={{ padding: '6rem 0' }}>
      <div className="container">
        <h2 className="text-center mb-12">❓ Frequently Asked Questions</h2>
        <div style={{ maxWidth: '800px', margin: '0 auto' }}>
          {faqs.map((f, i) => (
            <div key={i} style={{ marginBottom: '2rem', padding: '2rem', background: 'var(--color-bg-elevated)', borderRadius: '16px' }}>
              <h4 style={{ color: 'var(--ifm-color-primary)', margin: '0 0 1rem 0' }}>{f.q}</h4>
              <p style={{ margin: 0, color: 'var(--color-text-secondary)' }}>{f.a}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- Main Layout Sections ---

function Hero() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}><div className={styles.gridLines} /></div>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>2026 Frontier Edition</div>
          <Heading as="h1" className={clsx(styles.heroTitle, styles.gradientText)}>Physical AI & Humanoid Robotics</Heading>
          <p className={styles.heroSubtitle}>Build the Nervous System of Tomorrow's Machines. Master Kilted Kaiju & Blackwell AI.</p>
          <div className={styles.heroCta}>
            <Link className={clsx('button button--lg', styles.primaryButton)} to="/docs/M1/C1/m1-c1-s1">Start Learning →</Link>
          </div>
        </div>
      </div>
    </header>
  );
}

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
        <h2 className="text-center mb-12">Complete Curriculum</h2>
        <div className={styles.modulesGrid}>
          {modules.map((m, i) => (
            <div key={i} className={styles.moduleCard}>
              <div className={styles.moduleNumber} style={{ background: m.c }}>{m.n}</div>
              <div className={styles.moduleContent}>
                <h3 className={styles.moduleTitle}>{m.t}</h3>
                <p className={styles.moduleDescription}>{m.d}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactNode {
  useEffect(() => {
    const handleScroll = () => {
      document.querySelectorAll('.section-animate').forEach((s) => {
        if (s.getBoundingClientRect().top < window.innerHeight * 0.75) s.classList.add('section-visible');
      });
    };
    window.addEventListener('scroll', handleScroll);
    handleScroll();
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <Layout title="Physical AI Book" description="2026 Robotics Guide">
      <Hero />
      <YouTubeVideoGrid />

      <section className="container py-16">
        <h2 className="text-center mb-12">🤖 Interactive 3D Robot Viewer</h2>
        <RobotViewer />
      </section>

      <ModulesGrid />
      <LearningPathTimeline />

      <section className="container py-16">
        <h2 className="text-center mb-8">💻 Live Development Demo</h2>
        <div style={{ height: '700px', width: '100%', overflow: 'hidden', borderRadius: '16px' }}>
          <VscodeInterface />
        </div>
        <div className="text-center mt-8">
           <Link className="button button--secondary button--lg" to="/docs/M1/C1/m1-c1-s1">Try in Browser</Link>
        </div>
      </section>

      <RobotComparisonTable />
      <StatsDashboard />
      <TechStackShowcase />
      <FAQSection />
      <NewsletterSignup />

      <section style={{ padding: '6rem 0', textAlign: 'center' }}>
        <div className="container">
          <h2 className={styles.gradientText}>Ready to Build the Future?</h2>
          <Link className={clsx('button button--lg', styles.primaryButton)} to="/docs/M1/C1/m1-c1-s1">Get Started Today</Link>
        </div>
      </section>

      <button className={styles.quickStartButton} onClick={() => window.scrollTo({top: 0, behavior: 'smooth'})}>🚀</button>
    </Layout>
  );
}
