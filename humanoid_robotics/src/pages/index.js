


import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--6', styles.heroText)}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Get Started
              </Link>
            </div>
          </div>
          <div className={clsx('col col--6', styles.heroImageContainer)}>
            <img
              className={styles.heroImage}
              src={useBaseUrl('/img/book_icon.svg')}
              alt="Hero Image" />
          </div>
        </div>
      </div>
    </header>
  );
}

function QuoteSection() {
    return (
        <section className={styles.quoteSection}>
            <div className="container">
                <div className="row">
                    <div className="col col--8 col--offset-2">
                        <blockquote className={styles.quote}>
                            "Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space."
                        </blockquote>
                        <cite className={styles.quoteCite}>&mdash; Why Physical AI Matters</cite>
                    </div>
                </div>
            </div>
        </section>
    );
}

function CallToActionSection() {
    return (
        <section className={styles.ctaSection}>
            <div className="container text--center">
                <Heading as="h2">Ready to Dive In?</Heading>
                <p>Explore the modules and start building the future of robotics today.</p>
                <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    Begin Your Journey
                </Link>
            </div>
        </section>
    );
}


export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A book on Physical AI & Humanoid Robotics, covering ROS 2, Gazebo, and NVIDIA Isaac.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <QuoteSection />
        <CallToActionSection />
      </main>
    </Layout>
  );
}