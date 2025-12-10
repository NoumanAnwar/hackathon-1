import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';
import Chatbox from '@site/src/components/Chatbox'; // Import the Chatbox component

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/intro">
            Start Learning 🚀
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/module1/chapter1">
            Begin Reading 📖
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book - From Pixels to Actions">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>4 Comprehensive Modules</h3>
                  <p>From ROS 2 fundamentals to Vision-Language-Action systems</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>AI-Powered Learning</h3>
                  <p>Integrated RAG chatbot for personalized learning experience</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Advanced Features</h3>
                  <p>Personalization, multi-language support, and more</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={clsx(styles.featuresSection, "container")}>
          <HomepageFeatures />
        </section>

        {/* Add the Chatbox component here */}
        <section className={styles.chatboxSection}>
          <div className="container padding-horiz--md">
            <h2 className="text--center padding-bottom--md">Ask the Physical AI & Humanoid Robotics RAG Chatbot</h2>
            <div style={{ maxWidth: '800px', margin: '0 auto' }}>
              <Chatbox />
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
