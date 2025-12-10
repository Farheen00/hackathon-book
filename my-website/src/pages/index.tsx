import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">
          A Complete Guide to Embodied Intelligence, Digital Twins, and Humanoid Robot Control
        </p>

        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module-1-ros2-fundamentals">
            Start Learning
          </Link>

          <Link
            className="button button--primary button--lg"
            to="/chatbot"
            style={{ marginLeft: '1rem' }}>
            Open RAG Chatbot
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModulesOverview() {
  const modules = [
    {
      title: 'Module 1: ROS 2',
      description: 'Learn the fundamentals of Robot Operating System 2 for building robotic applications.',
      link: '/docs/module-1-ros2-fundamentals',
    },
    {
      title: 'Module 2: Digital Twin',
      description: 'Understand how to create and work with digital twin systems for robotics.',
      link: '/docs/module-2-digital-twin',
    },
    {
      title: 'Module 3: NVIDIA Isaac',
      description: 'Master NVIDIA Isaac for AI-powered robotic control and simulation.',
      link: '/docs/module-3-isaac-ai-brain',
    },
    {
      title: 'Module 4: VLA Systems',
      description: 'Explore Vision-Language-Action systems for advanced robotic perception.',
      link: '/docs/tutorial-basics/module-4-vla-integration',
    },
    {
      title: 'Capstone Project',
      description: 'Apply your knowledge in a comprehensive humanoid robotics project.',
      link: '/docs/capstone',
    },
  ];

  return (
    <section className={styles.modulesOverview} id="modules-overview">
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className="text--center margin-bottom--lg">
              Modules Overview
            </Heading>
          </div>
        </div>
        <div className="row">
          {modules.map((module, index) => (
            <div className="col col--4 margin-bottom--lg" key={index}>
              <div className="card">
                <div className="card__body text--center">
                  <Heading as="h3">{module.title}</Heading>
                  <p>{module.description}</p>
                  <Link to={module.link} className="button button--primary button--block">
                    Explore
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ChatbotSection() {
  return (
    <section className={styles.chatbotSection} id="chatbot-section">
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--12 text--center">
            <Heading as="h2">RAG Chatbot</Heading>
            <p>Ask questions about any part of the book using our RAG Chatbot</p>
            <Link className="button button--primary button--lg" to="/chatbot">
              Open Chatbot
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout title="Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ModulesOverview />
        <ChatbotSection />
      </main>
    </Layout>
  );
}